import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "site1_v12_terrain_coarse_release.py"
)
SPEC = importlib.util.spec_from_file_location(
    "site1_v12_terrain_coarse_release", SCRIPT
)
TERRAIN = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = TERRAIN
SPEC.loader.exec_module(TERRAIN)


# The production Scene1 schema has 38 properties: three uchar colours, one
# double property, and 34 float properties.  Names are unimportant to the
# byte-preservation proof beyond XYZ being present.
DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("scalar_precise", "<f8"),
    ]
    + [(f"scalar_{index:02d}", "<f4") for index in range(28)]
)
assert len(DTYPE.names) == 38


def records(count, *, offset):
    value = np.zeros(count, DTYPE)
    index = np.arange(count, dtype=np.float32)
    value["x"] = offset + index * 0.006
    value["y"] = 810.0 + index * 0.007
    value["z"] = 2.0 + index * 0.001
    value["red"] = 30 + index.astype(np.uint8)
    value["green"] = 60 + index.astype(np.uint8)
    value["blue"] = 90 + index.astype(np.uint8)
    value["nx"] = 0.01
    value["ny"] = -0.02
    value["nz"] = 0.999
    value["scalar_precise"] = offset * 1000.0 + index
    for scalar_index in range(28):
        value[f"scalar_{scalar_index:02d}"] = (
            offset * 10.0 + scalar_index + index * 0.01
        )
    return value


PLY_TYPE = {
    "<f4": "float",
    "<f8": "double",
    "|u1": "uchar",
}


def write_ply(path, value, *, comment):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"comment {comment}",
        f"element vertex {len(value)}",
    ]
    for name in value.dtype.names:
        lines.append(f"property {PLY_TYPE[value.dtype.fields[name][0].str]} {name}")
    lines.append("end_header")
    with path.open("xb") as handle:
        handle.write(("\n".join(lines) + "\n").encode("ascii"))
        handle.write(value.tobytes())
    return path


def load_records(path):
    info = TERRAIN.water_release.inspect_ply(path)
    return np.memmap(
        path,
        dtype=info.dtype,
        mode="r",
        offset=info.offset,
        shape=(info.count,),
    )


class Fixture:
    def __init__(self, root, *, property_count=38):
        self.root = Path(root).resolve()
        self.data = self.root / "Data" / "Scene1"
        self.patch = self.data / "PatchRefinement"
        self.run = self.patch / "terrain-v12"
        self.patch.mkdir(parents=True)
        self.downsample = self.root / "bin" / "cleanmesh_spatial_downsample"
        self.downsample.parent.mkdir()
        self.downsample.write_bytes(b"fixture native downsampler")
        self.fine = {}
        self.previous = {}
        for layer_index, layer in enumerate(TERRAIN.LAYERS):
            source = records(9, offset=100.0 + layer_index * 100.0)
            if property_count != 38:
                names = source.dtype.names[:property_count]
                reduced = np.zeros(
                    len(source),
                    np.dtype(
                        [
                            (name, source.dtype.fields[name][0])
                            for name in names
                        ]
                    ),
                )
                for name in names:
                    reduced[name] = source[name]
                source = reduced
            self.fine[layer] = write_ply(
                self.data / TERRAIN.FINE_BY_LAYER[layer],
                source,
                comment=f"{layer} native fine",
            )
            self.previous[layer] = write_ply(
                self.data / TERRAIN.COARSE_BY_LAYER[layer],
                source[[1, 5]].copy(),
                comment=f"{layer} previous independent coarse",
            )
        self.args = SimpleNamespace(
            data_dir=self.data,
            run_dir=self.run,
            release_dir=None,
            downsample=self.downsample,
            chunk_points=37,
        )
        self.commands = []

    def runner(self, command, *, check):
        self.commands.append(list(command))
        self.assert_command(command)
        fine = Path(command[command.index("--input") + 1])
        output = Path(command[command.index("--output") + 1])
        report = Path(command[command.index("--report") + 1])
        source = load_records(fine)
        coarse = np.asarray(source[[0, 2, 4, 8]]).copy()
        write_ply(output, coarse, comment="native exact full-record subset")
        info_fine = TERRAIN.water_release.inspect_ply(fine)
        info_coarse = TERRAIN.water_release.inspect_ply(output)
        report.write_text(
            json.dumps(
                {
                    "input": str(fine.resolve()),
                    "output": str(output.resolve()),
                    "method": "greedy_spatial_minimum_distance",
                    "minimum_spacing_m": 0.005,
                    "source_points": info_fine.count,
                    "output_points": info_coarse.count,
                    "record_stride": info_fine.stride,
                    "occupied_cells": info_coarse.count,
                    "non_finite_positions": 0,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        return SimpleNamespace(returncode=0)

    def assert_command(self, command):
        assert command[command.index("--spacing") + 1] == "0.005"
        assert command[command.index("--chunk-points") + 1] == "37"
        assert "--priority-scan-id" not in command
        assert "--force" not in command

    def build(self):
        return TERRAIN.build(self.args, command_runner=self.runner)


class FineFirstBuildTests(unittest.TestCase):
    def test_build_downsamples_each_semantic_layer_independently(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            previous_hashes = {
                layer: TERRAIN.file_fingerprint(path)["sha256"]
                for layer, path in fixture.previous.items()
            }
            result = fixture.build()
            self.assertTrue(result["built"])
            self.assertTrue(result["candidate_only"])
            self.assertTrue(result["semantic_layers_downsampled_separately"])
            self.assertFalse(result["coarse_scalar_recalculation"])
            self.assertEqual(len(fixture.commands), 3)
            for layer, command in zip(TERRAIN.LAYERS, fixture.commands):
                self.assertEqual(
                    command[command.index("--input") + 1],
                    str(fixture.data / TERRAIN.FINE_BY_LAYER[layer]),
                )
                self.assertEqual(
                    command[command.index("--output") + 1],
                    str(fixture.run / "stages" / layer.lower() / f"Site1-{layer}-5mm.candidate.ply"),
                )
                self.assertEqual(
                    TERRAIN.file_fingerprint(fixture.previous[layer])["sha256"],
                    previous_hashes[layer],
                )

    def test_candidates_preserve_all_38_fields_as_exact_ordered_records(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            result = fixture.build()
            for layer in TERRAIN.LAYERS:
                stage = result["layers"][layer]["stage"]
                self.assertEqual(stage["schema"]["property_count"], 38)
                subset = stage["exact_ordered_subsequence"]
                self.assertEqual(
                    subset["relation"],
                    "byte-exact ordered full-record subsequence",
                )
                self.assertEqual(subset["matched_points"], 4)

    def test_build_rejects_a_fine_source_with_fewer_than_38_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory, property_count=37)
            with self.assertRaisesRegex(RuntimeError, "exactly 38 properties"):
                fixture.build()
            for layer in TERRAIN.LAYERS:
                self.assertTrue(fixture.previous[layer].exists())

    def test_verify_rejects_candidate_payload_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.build()
            candidate = fixture.run / "stages" / "sand" / "Site1-SAND-5mm.candidate.ply"
            with candidate.open("r+b") as handle:
                handle.seek(-1, os.SEEK_END)
                value = handle.read(1)
                handle.seek(-1, os.SEEK_END)
                handle.write(bytes([value[0] ^ 0x01]))
            with self.assertRaisesRegex(RuntimeError, "fingerprint drift"):
                TERRAIN.verify(fixture.args)


class ReleaseTransactionTests(unittest.TestCase):
    def test_install_and_restore_are_atomic_and_keep_candidates(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.build()
            previous = {
                layer: TERRAIN.file_fingerprint(fixture.previous[layer])
                for layer in TERRAIN.LAYERS
            }
            candidate_paths = {
                layer: fixture.run / "stages" / layer.lower() / f"Site1-{layer}-5mm.candidate.ply"
                for layer in TERRAIN.LAYERS
            }
            candidates = {
                layer: TERRAIN.file_fingerprint(path)
                for layer, path in candidate_paths.items()
            }
            with mock.patch.object(TERRAIN, "refuse_running_app"):
                installed = TERRAIN.install(fixture.args)
            self.assertTrue(installed["installed"])
            for layer in TERRAIN.LAYERS:
                active = TERRAIN.file_fingerprint(fixture.previous[layer])
                self.assertTrue(TERRAIN._same_content(active, candidates[layer], ply=True))
                self.assertTrue(candidate_paths[layer].exists())
                self.assertNotEqual(active["inode"], candidates[layer]["inode"])
            with mock.patch.object(TERRAIN, "refuse_running_app"):
                restored = TERRAIN.restore(fixture.args)
            self.assertTrue(restored["restored"])
            for layer in TERRAIN.LAYERS:
                active = TERRAIN.file_fingerprint(fixture.previous[layer])
                self.assertTrue(TERRAIN._same_content(active, previous[layer], ply=True))
                self.assertTrue(candidate_paths[layer].exists())
            result = TERRAIN.verify(fixture.args)
            self.assertEqual(result["status"], "restored")

    def test_mid_install_failure_rolls_all_layers_back(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.build()
            previous = {
                layer: TERRAIN.file_fingerprint(fixture.previous[layer])
                for layer in TERRAIN.LAYERS
            }
            original_replace = TERRAIN._durable_replace
            failed = False

            def fail_once(source, destination):
                nonlocal failed
                if (
                    not failed
                    and source == fixture.previous["ROCK"]
                    and destination.name == TERRAIN.COARSE_BY_LAYER["ROCK"]
                    and destination.parent.name == "previous"
                ):
                    failed = True
                    raise OSError("injected ROCK archive failure")
                return original_replace(source, destination)

            with (
                mock.patch.object(TERRAIN, "refuse_running_app"),
                mock.patch.object(TERRAIN, "_durable_replace", side_effect=fail_once),
                self.assertRaisesRegex(OSError, "injected ROCK"),
            ):
                TERRAIN.install(fixture.args)
            for layer in TERRAIN.LAYERS:
                active = TERRAIN.file_fingerprint(fixture.previous[layer])
                self.assertTrue(TERRAIN._same_content(active, previous[layer], ply=True))
            manifest = TERRAIN._load_json(
                fixture.run / "release" / "manifest.json", "manifest"
            )[1]
            self.assertEqual(manifest["status"], "built")
            journals = list((fixture.run / "release" / "transactions").glob("*/journal.json"))
            self.assertEqual(len(journals), 1)
            journal = TERRAIN._load_json(journals[0], "journal")[1]
            self.assertEqual(journal["state"], "rolled-back")

    def test_verify_recovers_an_incomplete_archived_first_layer(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.build()
            paths = TERRAIN._coerce_paths(fixture.args)
            _, manifest = TERRAIN._read_manifest(paths)
            transaction = TERRAIN._new_transaction_dir(paths, "install")
            items, _ = TERRAIN._prepare_items(paths, manifest, transaction, "install")
            journal = {
                "schema_version": TERRAIN.JOURNAL_SCHEMA_VERSION,
                "operation": TERRAIN.JOURNAL_OPERATION,
                "created": TERRAIN._now(),
                "action": "install",
                "source_status": "built",
                "target_status": "installed",
                "state": "archived-current",
                "items": [
                    {
                        "layer": item.layer,
                        "canonical": str(item.canonical),
                        "replacement": str(item.replacement),
                        "archive": str(item.archive),
                        "expected_current": dict(item.expected_current),
                        "expected_replacement": dict(item.expected_replacement),
                    }
                    for item in items
                ],
                "events": [],
            }
            TERRAIN._journal_write(transaction / "journal.json", journal)
            TERRAIN._durable_replace(items[0].canonical, items[0].archive)
            with mock.patch.object(TERRAIN, "refuse_running_app"):
                result = TERRAIN.verify(fixture.args)
            self.assertEqual(
                result["recovered_transactions"][0]["outcome"],
                "rolled-back-after-recovery",
            )
            self.assertEqual(result["status"], "built")
            for layer in TERRAIN.LAYERS:
                self.assertTrue(fixture.previous[layer].exists())


class FilesystemSafetyTests(unittest.TestCase):
    def test_hard_linked_fine_source_is_rejected_before_run_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            os.link(fixture.fine["SAND"], fixture.root / "sand-hardlink.ply")
            with self.assertRaisesRegex(RuntimeError, "multiple hard links"):
                fixture.build()
            self.assertFalse(fixture.run.exists())

    def test_symlinked_canonical_is_rejected_before_run_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            original = fixture.previous["VEG"]
            moved = original.with_name("VEG-real.ply")
            original.rename(moved)
            original.symlink_to(moved)
            with self.assertRaisesRegex(RuntimeError, "symlink"):
                fixture.build()
            self.assertFalse(fixture.run.exists())

    def test_hard_linked_lock_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            lock = fixture.data / ".site1-v12-terrain-coarse-release.lock"
            lock.write_text("existing\n")
            os.link(lock, fixture.root / "lock-hardlink")
            with self.assertRaisesRegex(RuntimeError, "private regular file"):
                fixture.build()
            self.assertFalse(fixture.run.exists())

    def test_manifest_run_directory_rebinding_fails_before_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.build()
            manifest_path = fixture.run / "release" / "manifest.json"
            manifest = TERRAIN._load_json(manifest_path, "manifest")[1]
            manifest["run_dir"] = str(fixture.patch / "different-run")
            TERRAIN._atomic_json(manifest_path, manifest, overwrite=True)
            with self.assertRaisesRegex(RuntimeError, "run directory binding"):
                TERRAIN.verify(fixture.args)


if __name__ == "__main__":
    unittest.main()
