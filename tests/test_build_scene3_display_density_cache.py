from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from scripts import build_scene3_display_density_cache as cache_builder


TEST_DTYPE = np.dtype(
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
        ("scalar_Intensity", "<f8"),
        ("scalar_ScanID", "<f4"),
        ("scalar_Roughness", "<f4"),
    ]
)

TEST_PROPERTIES = (
    ("float", "x"),
    ("float", "y"),
    ("float", "z"),
    ("uchar", "red"),
    ("uchar", "green"),
    ("uchar", "blue"),
    ("float", "nx"),
    ("float", "ny"),
    ("float", "nz"),
    ("double", "scalar_Intensity"),
    ("float", "scalar_ScanID"),
    ("float", "scalar_Roughness"),
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_ply(path: Path, records: np.ndarray) -> None:
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        "comment deterministic cache test fixture",
        f"element vertex {records.size}",
    ]
    lines.extend(f"property {type_name} {name}" for type_name, name in TEST_PROPERTIES)
    lines.append("end_header")
    with path.open("wb") as output:
        output.write(("\n".join(lines) + "\n").encode("ascii"))
        output.write(records.tobytes())


def _read_records(path: Path) -> np.ndarray:
    description = cache_builder.read_ply_description(path)
    with path.open("rb") as source:
        source.seek(description.data_offset)
        return np.frombuffer(
            source.read(description.vertex_count * description.dtype.itemsize),
            dtype=description.dtype,
        ).copy()


def _fixture_records() -> np.ndarray:
    records = np.zeros(12, dtype=TEST_DTYPE)
    # Three 5 mm cells with populations 6, 4, and 2.  A five-point target
    # exercises proportional multi-strata output instead of one-per-cell.
    records["x"] = np.asarray(
        [0.0001, 0.0004, 0.0008, 0.0012, 0.0018, 0.0022,
         0.0051, 0.0056, 0.0062, 0.0070,
         0.0101, 0.0110],
        dtype=np.float32,
    )
    records["y"] = np.linspace(0.0001, 0.0012, records.size, dtype=np.float32)
    records["z"] = 1.0
    records["red"] = np.arange(records.size, dtype=np.uint8) * 10
    records["green"] = 100
    records["blue"] = 200
    records["nz"] = 1.0
    records["scalar_Intensity"] = np.arange(records.size, dtype=np.float64)
    records["scalar_ScanID"] = np.asarray([3, 3, 3, 4, 4, 5] * 2, dtype=np.float32)
    records["scalar_Roughness"] = np.linspace(0.0, 1.0, records.size, dtype=np.float32)
    return records


def _filter_fixture_records() -> np.ndarray:
    records = np.zeros(2, dtype=TEST_DTYPE)
    records["x"] = [0.0001, 0.0002]
    records["y"] = [0.0001, 0.0002]
    records["z"] = [1.0, 1.0001]
    records["red"] = [0, 255]
    records["green"] = [0, 255]
    records["blue"] = [0, 255]
    records["nz"] = [1.0, -1.0]
    records["scalar_Intensity"] = [1.0, 3.0]
    records["scalar_ScanID"] = [5.0, 3.0]
    records["scalar_Roughness"] = [np.nan, 4.0]
    return records


class DisplayDensityCacheBuilderTests(unittest.TestCase):
    def _write_roles(self, source_root: Path, records: np.ndarray) -> dict[str, tuple[str, int]]:
        source_root.mkdir(parents=True)
        proofs: dict[str, tuple[str, int]] = {}
        for role in cache_builder.ROLE_ORDER:
            path = source_root / f"Site3-{role}-1mm.ply"
            _write_ply(path, records)
            stat = path.stat()
            proofs[role] = (_sha256(path), stat.st_mtime_ns)
        return proofs

    def _build(
        self,
        source_root: Path,
        cache_root: Path,
        target: int,
        rgb_filter: str = cache_builder.RGB_FILTER_RENDERER_BYTE,
    ) -> Path:
        return cache_builder.build_cache(
            cache_builder.BuildConfig(
                source_root=source_root,
                cache_root=cache_root,
                targets={role: target for role in cache_builder.ROLE_ORDER},
                voxel_size_m=0.005,
                shard_count=4,
                seed=0,
                rgb_filter=rgb_filter,
                chunk_records=3,
            )
        )

    def test_complete_bundle_is_exact_deterministic_and_sources_stay_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            source_proofs = self._write_roles(source_root, _fixture_records())

            bundle_a = self._build(source_root, root / "cache-a", target=5)
            bundle_b = self._build(source_root, root / "cache-b", target=5)
            self.assertEqual(bundle_a.name, bundle_b.name)

            manifest = json.loads(
                (bundle_a / cache_builder.MANIFEST_NAME).read_text(encoding="utf-8")
            )
            self.assertTrue(manifest["complete"])
            self.assertEqual(manifest["bundle_fingerprint"], bundle_a.name)
            self.assertEqual(
                manifest["algorithm"]["id"], cache_builder.ALGORITHM_ID
            )
            self.assertEqual(
                manifest["algorithm"]["rgb_filter"],
                cache_builder.RGB_FILTER_RENDERER_BYTE,
            )
            self.assertEqual(
                [role["role"] for role in manifest["roles"]],
                list(cache_builder.ROLE_ORDER),
            )

            for role_record in manifest["roles"]:
                role = role_record["role"]
                output_a = bundle_a / role_record["output"]["file"]
                output_b = bundle_b / role_record["output"]["file"]
                description = cache_builder.read_ply_description(output_a)
                self.assertEqual(description.vertex_count, 5)
                self.assertEqual(output_a.read_bytes(), output_b.read_bytes())
                self.assertEqual(
                    role_record["source"]["sha256"], source_proofs[role][0]
                )
                self.assertEqual(
                    role_record["output"]["sha256"], _sha256(output_a)
                )
                # Every sampled location is one of the actual parent positions.
                child_x = set(_read_records(output_a)["x"].tolist())
                parent_x = set(_fixture_records()["x"].tolist())
                self.assertTrue(child_x.issubset(parent_x))
                output_x = _read_records(output_a)["x"]
                self.assertEqual(
                    [
                        int(np.count_nonzero(output_x < 0.005)),
                        int(
                            np.count_nonzero(
                                (output_x >= 0.005) & (output_x < 0.010)
                            )
                        ),
                        int(np.count_nonzero(output_x >= 0.010)),
                    ],
                    [2, 2, 1],
                )

            for role, (source_hash, source_mtime_ns) in source_proofs.items():
                source = source_root / f"Site3-{role}-1mm.ply"
                self.assertEqual(_sha256(source), source_hash)
                self.assertEqual(source.stat().st_mtime_ns, source_mtime_ns)

            pointer = json.loads(
                ((root / "cache-a") / cache_builder.ACTIVE_POINTER_NAME).read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(pointer["bundle_fingerprint"], bundle_a.name)
            self.assertEqual(
                pointer["manifest_sha256"],
                _sha256(bundle_a / cache_builder.MANIFEST_NAME),
            )

    def test_rgb_modes_normals_scanid_and_finite_means_are_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            self._write_roles(source_root, _filter_fixture_records())
            cache_root = root / "cache"
            byte_bundle = self._build(source_root, cache_root, target=1)
            pointer_path = cache_root / cache_builder.ACTIVE_POINTER_NAME
            settled_pointer = pointer_path.read_bytes()
            linear_bundle = self._build(
                source_root,
                cache_root,
                target=1,
                rgb_filter=cache_builder.RGB_FILTER_SRGB_LINEAR,
            )

            byte_point = _read_records(
                byte_bundle / "Scene3" / "Site3-ROCK-5mm.ply"
            )[0]
            linear_point = _read_records(
                linear_bundle / "Scene3" / "Site3-ROCK-5mm.ply"
            )[0]
            self.assertEqual(int(byte_point["red"]), 128)
            self.assertEqual(int(linear_point["red"]), 188)
            self.assertAlmostEqual(abs(float(byte_point["nz"])), 1.0, places=6)
            self.assertAlmostEqual(float(byte_point["scalar_ScanID"]), 3.0)
            self.assertAlmostEqual(float(byte_point["scalar_Roughness"]), 4.0)
            self.assertIn(
                float(byte_point["x"]),
                _filter_fixture_records()["x"].tolist(),
            )
            self.assertNotEqual(byte_bundle, linear_bundle)
            self.assertTrue((linear_bundle / cache_builder.MANIFEST_NAME).is_file())
            self.assertEqual(pointer_path.read_bytes(), settled_pointer)

    def test_failed_role_never_replaces_active_pointer(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            source_root.mkdir()
            for role in ("ROCK", "SAND"):
                _write_ply(
                    source_root / f"Site3-{role}-1mm.ply", _fixture_records()
                )
            cache_root = root / "cache"
            cache_root.mkdir()
            pointer = cache_root / cache_builder.ACTIVE_POINTER_NAME
            pointer.write_text(
                '{"schema_version":1,"bundle_fingerprint":"settled-old"}\n',
                encoding="utf-8",
            )
            original_pointer = pointer.read_bytes()

            with self.assertRaises(FileNotFoundError):
                self._build(source_root, cache_root, target=5)

            self.assertEqual(pointer.read_bytes(), original_pointer)
            self.assertFalse(any(cache_root.glob(".staging-*")))
            self.assertEqual(
                [entry.name for entry in cache_root.iterdir()],
                [cache_builder.ACTIVE_POINTER_NAME],
            )

    def test_cache_source_alias_and_insufficient_space_fail_before_staging(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            self._write_roles(source_root, _fixture_records())

            with self.assertRaisesRegex(ValueError, "must be disjoint"):
                self._build(source_root, source_root / "cache", target=5)
            self.assertFalse((source_root / "cache").exists())

            cache_root = root / "separate-cache"
            with mock.patch.object(
                cache_builder.shutil,
                "disk_usage",
                return_value=mock.Mock(free=1),
            ):
                with self.assertRaisesRegex(RuntimeError, "insufficient free space"):
                    self._build(source_root, cache_root, target=5)
            self.assertFalse(any(cache_root.glob(".staging-*")))

    def test_existing_fingerprint_is_fully_revalidated_before_reactivation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            self._write_roles(source_root, _fixture_records())
            cache_root = root / "cache"
            bundle = self._build(source_root, cache_root, target=5)
            output = bundle / "Scene3" / "Site3-ROCK-5mm.ply"
            original_stat = output.stat()
            with output.open("r+b") as stream:
                stream.seek(-1, os.SEEK_END)
                byte = stream.read(1)
                stream.seek(-1, os.SEEK_END)
                stream.write(bytes([byte[0] ^ 0x01]))
            os.utime(
                output,
                ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns),
            )
            pointer = cache_root / cache_builder.ACTIVE_POINTER_NAME
            pointer.write_text(
                '{"schema_version":1,"bundle_fingerprint":"prior-settled",'
                '"manifest_sha256":"proof"}\n',
                encoding="utf-8",
            )
            prior_pointer = pointer.read_bytes()

            with self.assertRaisesRegex(RuntimeError, "output hash changed"):
                self._build(source_root, cache_root, target=5)

            self.assertEqual(pointer.read_bytes(), prior_pointer)
            self.assertFalse(any(cache_root.glob(".staging-*")))

    def test_source_guard_detects_change_after_an_earlier_role_finishes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            self._write_roles(source_root, _fixture_records())
            path = source_root / "Site3-ROCK-1mm.ply"
            stat = path.stat()
            records = [
                {
                    "role": "ROCK",
                    "source": {
                        "path": str(path),
                        "size_bytes": stat.st_size,
                        "mtime_ns": stat.st_mtime_ns,
                    },
                }
            ]
            os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1))
            with self.assertRaisesRegex(RuntimeError, "changed after its role build"):
                cache_builder._assert_sources_unchanged(records)

    def test_normal_prefilter_does_not_blend_across_a_sharp_face(self) -> None:
        records = np.zeros(3, dtype=TEST_DTYPE)
        records["x"] = [0.0, 0.001, 0.002]
        records["nz"] = [1.0, 1.0, 0.0]
        records["nx"] = [0.0, 0.0, 1.0]
        output = cache_builder._aggregate_sorted_groups(
            records,
            np.asarray([0], dtype=np.int64),
            cache_builder.RGB_FILTER_RENDERER_BYTE,
        )
        self.assertAlmostEqual(float(output["nx"][0]), 0.0, places=6)
        self.assertAlmostEqual(float(output["nz"][0]), 1.0, places=6)

    def test_systematic_rounding_keeps_sparse_cells_eligible_and_exact(self) -> None:
        # Twenty singleton boundary cells share five fractional outputs while
        # one dense cell has an exact five-point base quota. A global
        # largest-remainder occupancy cutoff could discard the sparse class;
        # systematic wrapping selects exactly five of those singleton cells.
        counts = np.asarray([1] * 20 + [20], dtype=np.uint64)
        priorities = cache_builder._mix64(
            np.arange(counts.size, dtype=np.uint64)
        )
        extras_a, extras_required = cache_builder._make_systematic_extras(
            counts,
            priorities,
            source_count=40,
            target_count=10,
            seed=1234,
        )
        extras_b, _ = cache_builder._make_systematic_extras(
            counts,
            priorities,
            source_count=40,
            target_count=10,
            seed=1234,
        )
        bases = (counts * np.uint64(10)) // np.uint64(40)
        quotas = bases + extras_a.astype(np.uint64)
        self.assertEqual(extras_required, 5)
        self.assertEqual(int(np.count_nonzero(extras_a[:20])), 5)
        self.assertEqual(int(np.sum(quotas, dtype=np.uint64)), 10)
        np.testing.assert_array_equal(extras_a, extras_b)


if __name__ == "__main__":
    unittest.main()
