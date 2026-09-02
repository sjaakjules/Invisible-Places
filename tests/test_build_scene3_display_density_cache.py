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
        ("scalar_A_R_Downhill_Azimuth_deg", "<f4"),
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
    ("float", "scalar_A_R_Downhill_Azimuth_deg"),
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
    records["scalar_A_R_Downhill_Azimuth_deg"] = [179.0, -179.0]
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
            self.assertEqual(
                manifest["schema_version"],
                cache_builder.SCHEMA_VERSION,
            )
            self.assertEqual(manifest["bundle_fingerprint"], bundle_a.name)
            self.assertEqual(
                manifest["algorithm"]["id"], cache_builder.ALGORITHM_ID
            )
            self.assertEqual(
                manifest["algorithm"]["version"],
                cache_builder.ALGORITHM_VERSION,
            )
            self.assertEqual(
                manifest["algorithm"]["position_policy"],
                cache_builder.POSITION_POLICY,
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
                # Every sampled XYZ tuple is one actual parent, never a
                # synthetic centroid assembled from independent axes.
                output_records = _read_records(output_a)
                child_xyz = {
                    (float(point["x"]), float(point["y"]), float(point["z"]))
                    for point in output_records
                }
                parent_xyz = {
                    (float(point["x"]), float(point["y"]), float(point["z"]))
                    for point in _fixture_records()
                }
                self.assertTrue(child_xyz.issubset(parent_xyz))
                output_x = output_records["x"]
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
                analysis = role_record["analysis"]
                self.assertEqual(
                    analysis["schema_version"],
                    cache_builder.ANALYSIS_SCHEMA_VERSION,
                )
                self.assertEqual(
                    analysis["weight_channels"],
                    [
                        "density_continuity",
                        "prefer_lower",
                        "prefer_upper",
                        "soft_separation",
                    ],
                )
                sidecars = {
                    name: bundle_a / analysis[name]["file"]
                    for name in (
                        "fine_to_coarse",
                        "coarse_parent_count",
                        "fine_stability_weights",
                        "coarse_stability_weights",
                    )
                }
                for name, sidecar in sidecars.items():
                    self.assertTrue(sidecar.is_file(), name)
                    self.assertEqual(
                        analysis[name]["sha256"],
                        _sha256(sidecar),
                    )
                links = np.fromfile(sidecars["fine_to_coarse"], dtype="<u4")
                parents = np.fromfile(
                    sidecars["coarse_parent_count"], dtype="<u4"
                )
                fine_weights = np.fromfile(
                    sidecars["fine_stability_weights"], dtype="<u4"
                )
                coarse_weights = np.fromfile(
                    sidecars["coarse_stability_weights"], dtype="<u4"
                )
                self.assertEqual(links.size, _fixture_records().size)
                self.assertEqual(parents.size, 5)
                self.assertEqual(fine_weights.size, links.size)
                self.assertEqual(coarse_weights.size, parents.size)
                linked = links != cache_builder.SOURCE_INDEX_SENTINEL
                self.assertTrue(np.all(links[linked] < coarse_weights.size))
                np.testing.assert_array_equal(
                    fine_weights[linked], coarse_weights[links[linked]]
                )
                self.assertEqual(
                    int(np.sum(parents, dtype=np.uint64)),
                    int(np.count_nonzero(linked)),
                )
                if role == "VEG":
                    self.assertTrue(
                        np.all(coarse_weights == np.uint32(0xFFFFFFFF))
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
            self.assertAlmostEqual(
                abs(float(byte_point["scalar_A_R_Downhill_Azimuth_deg"])),
                180.0,
                places=4,
            )
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

    def test_quota_one_uses_nearest_xyz_centroid_parent_without_refiltering(
        self,
    ) -> None:
        records = np.zeros(3, dtype=TEST_DTYPE)
        records["x"] = [0.003, 0.004, 0.007]
        records["y"] = [0.003, 0.006, 0.004]
        records["z"] = [0.003, 0.004, 0.006]
        records["red"] = [10, 50, 90]
        records["green"] = [20, 60, 100]
        records["blue"] = [30, 70, 110]
        records["nz"] = 1.0
        records["scalar_Intensity"] = [1.0, 3.0, 8.0]
        records["scalar_ScanID"] = [7.0, 3.0, 3.0]
        records["scalar_Roughness"] = [2.0, np.nan, 4.0]

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "sources"
            self._write_roles(source_root, records)
            bundle = self._build(source_root, root / "cache", target=1)
            output = _read_records(
                bundle / "Scene3" / "Site3-ROCK-5mm.ply"
            )[0]

            expected_xyz = tuple(
                float(records[name][1]) for name in ("x", "y", "z")
            )
            actual_xyz = tuple(float(output[name]) for name in ("x", "y", "z"))
            self.assertEqual(actual_xyz, expected_xyz)
            self.assertIn(
                actual_xyz,
                {
                    tuple(float(point[name]) for name in ("x", "y", "z"))
                    for point in records
                },
            )

            role_seed = int.from_bytes(b"ROCK", "little")
            order = np.argsort(
                cache_builder._point_priority(records, role_seed),
                kind="stable",
            )
            stable_records = records[order]
            legacy_filtered = cache_builder._aggregate_sorted_groups(
                stable_records,
                np.asarray([0], dtype=np.int64),
                cache_builder.RGB_FILTER_RENDERER_BYTE,
            )[0]
            self.assertNotEqual(
                tuple(float(legacy_filtered[name]) for name in ("x", "y", "z")),
                expected_xyz,
            )
            for name in TEST_DTYPE.names or ():
                if name not in ("x", "y", "z"):
                    self.assertEqual(
                        output[name].tobytes(),
                        legacy_filtered[name].tobytes(),
                    )

    def test_centroid_distance_tie_uses_stable_hash_independent_of_source_order(
        self,
    ) -> None:
        records = np.zeros(2, dtype=TEST_DTYPE)
        records["x"] = [0.003, 0.007]
        records["y"] = [0.003, 0.007]
        records["z"] = [0.003, 0.007]
        records["red"] = [20, 80]
        records["green"] = [30, 90]
        records["blue"] = [40, 100]
        records["nz"] = 1.0
        records["scalar_Intensity"] = [2.0, 6.0]
        records["scalar_ScanID"] = 3.0
        records["scalar_Roughness"] = [1.0, 5.0]

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_a = root / "sources-a"
            source_b = root / "sources-b"
            self._write_roles(source_a, records)
            self._write_roles(source_b, records[::-1])
            bundle_a = self._build(source_a, root / "cache-a", target=1)
            bundle_b = self._build(source_b, root / "cache-b", target=1)
            output_a = bundle_a / "Scene3" / "Site3-ROCK-5mm.ply"
            output_b = bundle_b / "Scene3" / "Site3-ROCK-5mm.ply"
            self.assertEqual(_sha256(output_a), _sha256(output_b))

            role_seed = int.from_bytes(b"ROCK", "little")
            priorities = cache_builder._point_priority(records, role_seed)
            expected = records[int(np.argmin(priorities))]
            actual = _read_records(output_a)[0]
            self.assertEqual(
                tuple(float(actual[name]) for name in ("x", "y", "z")),
                tuple(float(expected[name]) for name in ("x", "y", "z")),
            )

    def test_quota_greater_than_one_keeps_strata_attributes_count_and_hash(
        self,
    ) -> None:
        records = _fixture_records()[:4]
        role_seed = int.from_bytes(b"ROCK", "little")
        order = np.argsort(
            cache_builder._point_priority(records, role_seed),
            kind="stable",
        )
        stable_records = records[order]
        expected = cache_builder._aggregate_sorted_groups(
            stable_records,
            np.asarray([0, 2], dtype=np.int64),
            cache_builder.RGB_FILTER_RENDERER_BYTE,
        )

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "Site3-ROCK-1mm.ply"
            _write_ply(source, records)
            description = cache_builder.read_ply_description(source)
            shard = root / "shard.bin"
            shard.write_bytes(stable_records.tobytes())
            output = cache_builder._aggregate_shard(
                shard,
                description,
                cache_builder.BuildConfig(
                    source_root=root,
                    cache_root=root / "cache",
                    targets={role: 2 for role in cache_builder.ROLE_ORDER},
                    voxel_size_m=0.005,
                    shard_count=1,
                    seed=0,
                    chunk_records=4,
                ),
                target_count=2,
                systematic_extras=np.asarray([False]),
            )

        self.assertEqual(output.size, 2)
        self.assertEqual(output.tobytes(), expected.tobytes())
        self.assertEqual(
            hashlib.sha256(output.tobytes()).hexdigest(),
            hashlib.sha256(expected.tobytes()).hexdigest(),
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

    def test_surface_modes_choose_density_lower_upper_and_role_specific_soft_winner(
        self,
    ) -> None:
        cell_x = np.zeros(5, dtype=np.int64)
        cell_y = np.zeros(5, dtype=np.int64)
        z = np.asarray([0.000, 0.002, 0.020, 0.022, 0.060], dtype=np.float64)
        # The lower scan is denser than the upper one; the middle scan is a
        # nearby rival and should receive fractional rather than binary weight.
        parents = np.asarray([8, 8, 2, 2, 10], dtype=np.uint32)
        recession = np.asarray([0.10, 0.11, 0.50, 0.51, 0.90], dtype=np.float64)

        sand = cache_builder._surface_analysis_weights_for_sorted(
            cell_x, cell_y, z, parents, recession, "SAND"
        )
        rock = cache_builder._surface_analysis_weights_for_sorted(
            cell_x, cell_y, z, parents, recession, "ROCK"
        )
        channels = lambda packed: np.column_stack(
            [
                ((packed >> np.uint32(shift)) & np.uint32(0xFF)).astype(
                    np.uint8
                )
                for shift in (0, 8, 16, 24)
            ]
        )
        sand_channels = channels(sand)
        rock_channels = channels(rock)

        # Density+continuity and Prefer Lower retain the denser lower scan.
        self.assertTrue(np.all(sand_channels[:2, 0] == 255))
        self.assertTrue(np.all(sand_channels[:2, 1] == 255))
        # Prefer Upper retains the highest scan and rejects a well-separated
        # lower scan.
        self.assertEqual(int(sand_channels[4, 2]), 255)
        self.assertEqual(int(sand_channels[0, 2]), 0)
        # The middle scan blends rather than toggles because its separation
        # lies between the close and fully-separated thresholds.
        self.assertGreater(int(sand_channels[2, 0]), 0)
        self.assertLess(int(sand_channels[2, 0]), 128)
        # SAND soft mode follows the density winner; ROCK soft mode follows
        # the upper surface for under-rock culling.
        np.testing.assert_array_equal(
            sand_channels[:, 3], sand_channels[:, 0]
        )
        np.testing.assert_array_equal(
            rock_channels[:, 3], rock_channels[:, 2]
        )


if __name__ == "__main__":
    unittest.main()
