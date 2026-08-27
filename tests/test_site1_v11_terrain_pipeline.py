from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "site1_v11_terrain_pipeline.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_terrain_pipeline", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PIPE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PIPE
SPEC.loader.exec_module(PIPE)


PLY_DTYPE = np.dtype(
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
        ("scalar_Composite", "<f4"),
        ("scalar_ScanID", "<f4"),
        ("scalar_A_R_Test", "<f4"),
    ],
    align=False,
)


PLY_TYPES = {
    "f4": "float",
    "f8": "double",
    "u1": "uchar",
    "i1": "char",
    "i2": "short",
    "u2": "ushort",
    "i4": "int",
    "u4": "uint",
}


COMBINED_NORMALIZATIONS = {
    metric: {"Fine": 2.0, "Medium": 4.0, "Broad": 8.0}
    for metric in PIPE.PHYSICAL_METRICS
}


DERIVED_DTYPE = np.dtype(
    list(PLY_DTYPE.descr)
    + [
        (f"scalar_A_R_{metric}_{scale}", "<f4")
        for metric in PIPE.PHYSICAL_METRICS
        for scale in PIPE.DERIVED_SCALES
    ]
    + [
        (f"scalar_A_R_{metric}_Combined", "<f4")
        for metric in PIPE.PHYSICAL_METRICS
    ]
    + [("scalar_A_R_RoughnessRelative_FineMedium", "<f4")]
    + [(name, "<f4") for name in PIPE.LOCAL_VISIBILITY_FIELDS],
    align=False,
)


def write_ply(path: Path, records: np.ndarray) -> None:
    with path.open("wb") as handle:
        handle.write(b"ply\nformat binary_little_endian 1.0\n")
        handle.write(f"element vertex {len(records)}\n".encode("ascii"))
        for name in records.dtype.names or ():
            code = records.dtype[name].str.lstrip("<>=|")
            handle.write(f"property {PLY_TYPES[code]} {name}\n".encode("ascii"))
        handle.write(b"end_header\n")
        records.tofile(handle)


def read_ply(path: Path) -> np.ndarray:
    layout = PIPE.terrain.inspect_fixed_stride_ply(path)
    memory = np.memmap(
        path,
        dtype=layout.dtype,
        mode="r",
        offset=layout.offset,
        shape=(layout.vertex_count,),
    )
    records = np.asarray(memory).copy()
    del memory
    return records


def plane_records(
    *,
    width_cells: int = 5,
    height_cells: int = 5,
    cell_m: float = 0.04,
    holes=(),
    origin=(0.0, 0.0),
) -> np.ndarray:
    holes = set(holes)
    xy = []
    offsets = (0.10, 0.30, 0.60, 0.85)
    for iy in range(height_cells):
        for ix in range(width_cells):
            if (ix, iy) in holes:
                continue
            for oy in offsets:
                for ox in offsets:
                    xy.append(
                        (
                            origin[0] + (ix + ox) * cell_m,
                            origin[1] + (iy + oy) * cell_m,
                        )
                    )
    xy = np.asarray(xy, np.float64)
    records = np.zeros(len(xy), PLY_DTYPE)
    records["x"] = xy[:, 0]
    records["y"] = xy[:, 1]
    records["z"] = 1.0 + 0.10 * xy[:, 0] - 0.05 * xy[:, 1]
    records["red"] = 120
    records["green"] = 100
    records["blue"] = 80
    normal = np.asarray([-0.10, 0.05, 1.0], np.float64)
    normal /= np.linalg.norm(normal)
    records["nx"], records["ny"], records["nz"] = normal
    records["scalar_Intensity"] = 50_000.0 + np.arange(len(records))
    records["scalar_Composite"] = 100.0
    records["scalar_ScanID"] = np.arange(len(records)) % 9
    records["scalar_A_R_Test"] = 0.25
    return records


def target() -> object:
    return PIPE.terrain.TerrainReviewTarget(
        target_id="terrain_mark",
        kind=PIPE.terrain.DeficitKind.MARKED,
        bbox=(0.0, 0.20, 0.0, 0.20),
        minimum_tier=PIPE.ConfidenceTier.SUPPORTED,
    )


def resolution(label: str, nominal: float) -> object:
    return PIPE.ResolutionParameters(
        label=label,
        nominal_spacing_m=nominal,
        deficit_cell_size_m=0.04,
        neighbourhood_radius_cells=1,
        minimum_expected_points=4.0,
        minimum_deficit_fraction=0.40,
        minimum_component_cells=2,
        support_radius_m=0.12,
        source_collar_m=0.15,
        cleanmesh_collar_m=0.15,
        property_donor_distance_m=0.20,
        maximum_proposals_per_target=10_000,
        geometry_batch_points=1_000,
        reference_energy_samples=100,
    )


def write_config(path: Path) -> None:
    value = {
        "marked_locations": {
            "image_test": [
                {
                    "id": "terrain_mark",
                    "review_bbox": [0.0, 0.20, 0.0, 0.20],
                    "world": [0.10, 0.10],
                    "screenshot_evidence": "Measured lower-density terrain region",
                    "written_action": (
                        "Add SAND/ROCK interstitial points with ScanID=10 only "
                        "where independent geometry agrees."
                    ),
                }
            ]
        }
    }
    path.write_text(json.dumps(value), encoding="utf-8")


def fake_cleanmesh_runner(
    executable: Path,
    input_path: Path,
    output_path: Path,
    report_path: Path,
    resolution_parameters,
    pipeline_parameters,
):
    del executable, resolution_parameters, pipeline_parameters
    source = read_ply(input_path)
    analysed_dtype = np.dtype(
        [(name, source.dtype[name]) for name in source.dtype.names or ()]
        + [("scalar_A_R_Test", "<f4")],
        align=False,
    )
    analysed = np.empty(len(source), analysed_dtype)
    for name in source.dtype.names or ():
        analysed[name] = source[name]
    analysed["scalar_A_R_Test"] = np.linspace(0.1, 0.9, len(source))
    write_ply(output_path, analysed)
    report_path.write_text(
        json.dumps({"success": True, "scope": "test-local-collar"}),
        encoding="utf-8",
    )
    return {"runner": "fake", "input_points": int(len(source))}


class DeficitAndRoleTests(unittest.TestCase):
    def test_bounded_residual_energy_matches_reference_when_support_is_below_cap(self):
        axis = np.linspace(-0.04, 0.04, 7)
        x, y = np.meshgrid(axis, axis)
        z = (
            1.0
            + 0.1 * x
            - 0.05 * y
            + 0.0002 * np.sin(80.0 * x) * np.cos(65.0 * y)
        )
        donors = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
        query = np.asarray([[0.0, 0.0], [0.01, -0.01]], np.float64)
        reference = PIPE.terrain.compute_local_residual_energy(
            query, donors, 1.0e-8, radius_m=0.06
        )
        bounded = PIPE.compute_bounded_local_residual_energy(
            query,
            donors,
            1.0e-8,
            radius_m=0.06,
            maximum_donors=64,
        )
        np.testing.assert_allclose(
            bounded.energy_m2, reference.energy_m2, rtol=1.0e-10, atol=1.0e-14
        )
        np.testing.assert_array_equal(bounded.donor_count, reference.donor_count)

    def test_role_requires_real_connected_deficit_and_proposals_stay_in_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sand_path = root / "sand.ply"
            rock_path = root / "rock.ply"
            holes = {(2, 2), (3, 2), (2, 3), (3, 3)}
            write_ply(sand_path, plane_records(holes=holes))
            write_ply(rock_path, plane_records(origin=(1.0, 1.0)))
            parameters = resolution("1mm", 0.005)
            targets = [target()]
            clouds = {
                "SAND": PIPE.collect_local_role_cloud(
                    sand_path,
                    role="SAND",
                    targets=targets,
                    collar_m=0.15,
                    chunk_records=17,
                ),
                "ROCK": PIPE.collect_local_role_cloud(
                    rock_path,
                    role="ROCK",
                    targets=targets,
                    collar_m=0.15,
                    chunk_records=17,
                ),
            }
            choice = PIPE.choose_supported_role(
                clouds, targets[0], parameters, dominance_ratio=1.0
            )
            self.assertEqual(choice.role, "SAND")
            selected = next(item for item in choice.assessments if item.role == "SAND")
            self.assertEqual(len(selected.deficit.components), 1)
            self.assertEqual(selected.deficit.components[0].cell_count, 4)
            proposals = PIPE.generate_irregular_deficit_proposals(
                selected.deficit,
                nominal_spacing_m=0.005,
                maximum_proposals=10_000,
                seed=7,
            )
            self.assertGreater(len(proposals.xy), 0)
            ix = np.floor(proposals.xy[:, 0] / 0.04).astype(int)
            iy = np.floor(proposals.xy[:, 1] / 0.04).astype(int)
            self.assertTrue(np.all(selected.deficit.candidate_mask[iy, ix]))
            self.assertLess(
                np.ptp(np.mod(proposals.xy[:, 0], 0.04)), 0.04
            )
            self.assertGreater(
                len(np.unique(np.round(np.mod(proposals.xy[:, 0], 0.04), 6))), 8
            )
            self.assertLess(
                np.count_nonzero(selected.deficit.candidate_mask),
                selected.deficit.candidate_mask.size,
            )

    def test_no_deficit_leaves_role_unresolved_instead_of_bbox_fill(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sand_path = root / "sand.ply"
            rock_path = root / "rock.ply"
            write_ply(sand_path, plane_records())
            write_ply(rock_path, plane_records(origin=(1.0, 1.0)))
            parameters = resolution("1mm", 0.005)
            targets = [target()]
            clouds = {
                role: PIPE.collect_local_role_cloud(
                    path,
                    role=role,
                    targets=targets,
                    collar_m=0.15,
                    chunk_records=23,
                )
                for role, path in (("SAND", sand_path), ("ROCK", rock_path))
            }
            choice = PIPE.choose_supported_role(clouds, targets[0], parameters)
            self.assertIsNone(choice.role)
            self.assertEqual(choice.reason, "no_connected_measured_density_deficit")


class CandidatePipelineTests(unittest.TestCase):
    def test_cross_scale_subset_is_maximal_and_ignores_floating_support(self):
        fine = np.asarray(
            [
                [0.000, 0.0, 1.00],
                [0.004, 0.0, 1.00],
                [0.000, 0.0, 1.05],
            ],
            np.float64,
        )
        # The return at z=1.05 represents only its own surface.  It must not
        # suppress either point on the real surface five centimetres below.
        existing = np.asarray([[0.000, 0.0, 1.05]], np.float64)
        selected = PIPE.select_cross_scale_fine_subset(
            fine,
            existing,
            np.asarray([1.0, 1.0, 1.0]),
            spacing_m=0.005,
            vertical_tolerance_m=0.012,
            distance_tolerance_m=1.0e-9,
            seed=7,
        )
        self.assertTrue(selected.represented_by_existing[2])
        self.assertFalse(np.any(selected.represented_by_existing[:2]))
        self.assertEqual(len(selected.selected_fine_indices), 1)
        self.assertIn(int(selected.selected_fine_indices[0]), (0, 1))
        self.assertLessEqual(
            float(np.max(selected.coverage_xy_distance_m)), 0.005 + 1.0e-9
        )
        self.assertLessEqual(
            float(np.max(selected.coverage_vertical_delta_m)), 0.012 + 1.0e-9
        )

    def test_local_cleanmesh_scalars_use_global_normalization_and_donor_visibility(self):
        initial = np.zeros(2, DERIVED_DTYPE)
        analysed = np.zeros(2, DERIVED_DTYPE)
        for index, field_name in enumerate(PIPE.LOCAL_VISIBILITY_FIELDS):
            initial[field_name] = (index + 1) / 10.0
            analysed[field_name] = 99.0
        component_values = {
            "MeanCurvature": (1.0, 2.0, 4.0),
            "CrossCurvature": (-1.0, -2.0, -4.0),
            "Recession": (20.0, 20.0, 20.0),
            "Roughness": (-1.0, 2.0, 4.0),
        }
        for metric, values in component_values.items():
            for scale, value in zip(PIPE.DERIVED_SCALES, values, strict=True):
                analysed[f"scalar_A_R_{metric}_{scale}"] = value
            analysed[f"scalar_A_R_{metric}_Combined"] = 77.0
        analysed["scalar_A_R_RoughnessRelative_FineMedium"] = 77.0

        audit = PIPE.postprocess_local_analysed_additions(
            analysed, initial, COMBINED_NORMALIZATIONS
        )

        for index, field_name in enumerate(PIPE.LOCAL_VISIBILITY_FIELDS):
            np.testing.assert_allclose(analysed[field_name], (index + 1) / 10.0)
        np.testing.assert_allclose(
            analysed["scalar_A_R_MeanCurvature_Combined"], 0.5
        )
        np.testing.assert_allclose(
            analysed["scalar_A_R_CrossCurvature_Combined"], -0.5
        )
        np.testing.assert_allclose(
            analysed["scalar_A_R_Recession_Combined"], 1.0
        )
        np.testing.assert_allclose(
            analysed["scalar_A_R_Roughness_Combined"], 0.275
        )
        np.testing.assert_allclose(
            analysed["scalar_A_R_RoughnessRelative_FineMedium"], 0.0
        )
        self.assertFalse(audit["local_histogram_normalization_retained"])
        self.assertEqual(len(audit["rebuilt_with_global_normalization"]), 5)

    def test_overlapping_targets_are_globally_arbitrated(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            holes = {(2, 2), (3, 2), (2, 3), (3, 3)}
            sand_path = root / "sand.ply"
            rock_path = root / "rock.candidate.ply"
            write_ply(sand_path, plane_records(holes=holes))
            write_ply(rock_path, plane_records(origin=(1.0, 1.0)))
            first = target()
            second = PIPE.terrain.TerrainReviewTarget(
                target_id="terrain_mark_overlap",
                kind=PIPE.terrain.DeficitKind.SCANNER,
                bbox=first.bbox,
                minimum_tier=PIPE.ConfidenceTier.SUPPORTED,
                centre_xy=(0.10, 0.10),
                search_radius_m=0.20,
            )
            executable = root / "fake-cleanmesh"
            executable.write_bytes(b"fake cleanmesh identity\n")
            executable.chmod(0o755)
            parameters = PIPE.PipelineParameters(
                fine=resolution("1mm", 0.005),
                coarse=resolution("5mm", 0.010),
                role_dominance_ratio=1.0,
                chunk_records=19,
                cleanmesh_chunk_points=100,
                cleanmesh_normalization_samples=100,
            )
            output = root / "terrain-1mm"
            result = PIPE.build_resolution_candidates(
                sand_path=sand_path,
                rock_base_path=rock_path,
                targets=(first, second),
                output_dir=output,
                cleanmesh_executable=executable,
                resolution=parameters.fine,
                pipeline=parameters,
                cleanmesh_runner=fake_cleanmesh_runner,
            )

            report = json.loads(Path(result.report_path).read_text(encoding="utf-8"))
            arbitration = report["global_arbitration"]["roles"]["SAND"]
            self.assertGreater(
                arbitration["locally_accepted"], arbitration["globally_accepted"]
            )
            self.assertGreater(arbitration["overlap_rejected"], 0)
            self.assertTrue(arbitration["spacing"]["measured_clearance_verified"])
            self.assertTrue(
                arbitration["spacing"]["candidate_pair_clearance_verified"]
            )
            ledger = np.load(output / arbitration["ledger_npz"])
            self.assertEqual(
                int(np.count_nonzero(ledger["globally_selected"])),
                arbitration["globally_accepted"],
            )
            self.assertEqual(result.addition_counts["SAND"], arbitration["globally_accepted"])

    def test_four_candidate_bundle_is_scanid10_local_and_hash_locked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            holes = {(2, 2), (3, 2), (2, 3), (3, 3)}
            sand_1mm = root / "Site1-SAND-1mm.ply"
            rock_1mm = root / "ROCK-1mm-obstruction-cleaned.candidate.ply"
            sand_5mm = root / "Site1-SAND-5mm.ply"
            rock_5mm = root / "ROCK-5mm-obstruction-cleaned.candidate.ply"
            write_ply(sand_1mm, plane_records(holes=holes))
            write_ply(sand_5mm, plane_records(holes=holes))
            write_ply(rock_1mm, plane_records(origin=(1.0, 1.0)))
            write_ply(rock_5mm, plane_records(origin=(1.0, 1.0)))
            config = root / "review.json"
            write_config(config)
            executable = root / "fake-cleanmesh"
            executable.write_bytes(b"fake cleanmesh identity\n")
            executable.chmod(0o755)
            normalization_manifest = root / "normalization.json"
            normalization_manifest.write_text(
                json.dumps(
                    {"rock_combined_normalizations": COMBINED_NORMALIZATIONS}
                ),
                encoding="utf-8",
            )
            normalization_hash = PIPE.sha256_path(normalization_manifest)
            destination = root / "terrain-candidates"
            parameters = PIPE.PipelineParameters(
                fine=resolution("1mm", 0.005),
                coarse=resolution("5mm", 0.010),
                role_dominance_ratio=1.0,
                chunk_records=19,
                cleanmesh_chunk_points=100,
                cleanmesh_normalization_samples=100,
            )
            source_hashes = {
                path: PIPE.sha256_path(path)
                for path in (sand_1mm, rock_1mm, sand_5mm, rock_5mm)
            }
            result = PIPE.build_terrain_candidates(
                sand_1mm_path=sand_1mm,
                rock_1mm_base_path=rock_1mm,
                sand_5mm_path=sand_5mm,
                rock_5mm_base_path=rock_5mm,
                config_path=config,
                cleanmesh_executable=executable,
                output_dir=destination,
                normalization_manifest_path=normalization_manifest,
                parameters=parameters,
                cleanmesh_runner=fake_cleanmesh_runner,
                require_scipy=False,
            )
            self.assertEqual(result.output_dir, destination.resolve())
            manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
            self.assertFalse(manifest["canonical_install_performed"])
            self.assertTrue(manifest["invariants"]["bbox_is_never_a_fill_mask"])
            self.assertEqual(
                manifest["combined_geometry_normalization"]["sha256"],
                normalization_hash,
            )
            self.assertTrue((destination / "normalization-manifest.json").is_file())
            self.assertTrue(
                manifest["invariants"]["all_three_surface_predictions_evaluated"]
            )
            self.assertEqual(manifest["parameters"], PIPE.asdict(parameters))
            self.assertEqual(
                set(manifest["implementation"]),
                {
                    "site1_v11_confidence.py",
                    "site1_v11_terrain.py",
                    "site1_v11_terrain_pipeline.py",
                },
            )
            additions_by_resolution = {}
            for label, sand_source, rock_source in (
                ("1mm", sand_1mm, rock_1mm),
                ("5mm", sand_5mm, rock_5mm),
            ):
                resolution_result = result.resolutions[label]
                sand_candidate = Path(resolution_result.candidate_paths["SAND"])
                rock_candidate = Path(resolution_result.candidate_paths["ROCK"])
                self.assertTrue(sand_candidate.is_file())
                self.assertTrue(rock_candidate.is_file())
                self.assertGreater(resolution_result.addition_counts["SAND"], 0)
                self.assertEqual(resolution_result.addition_counts["ROCK"], 0)
                sand_records = read_ply(sand_candidate)
                original_sand = read_ply(sand_source)
                np.testing.assert_array_equal(
                    sand_records[: len(original_sand)], original_sand
                )
                additions = sand_records[len(original_sand) :]
                additions_by_resolution[label] = additions
                self.assertTrue(np.all(additions["scalar_ScanID"] == 10))
                self.assertTrue(np.all(additions["scalar_Intensity"] >= 50_000.0))
                rock_records = read_ply(rock_candidate)
                np.testing.assert_array_equal(rock_records, read_ply(rock_source))
                report = json.loads(
                    Path(resolution_result.report_path).read_text(encoding="utf-8")
                )
                self.assertFalse(report["cleanmesh"]["full_cloud_analysis"])
                self.assertTrue(report["cleanmesh"]["span_identity_verified"])
                archive_path = Path(
                    resolution_result.addition_archive_paths["SAND"]
                )
                self.assertEqual(
                    PIPE.sha256_path(archive_path),
                    resolution_result.addition_archive_sha256["SAND"],
                )
                with np.load(archive_path, allow_pickle=False) as archive:
                    expected_keys = (
                        PIPE.AUTHORITATIVE_ARCHIVE_KEYS
                        if label == "1mm"
                        else PIPE.CROSS_SCALE_ARCHIVE_KEYS
                    )
                    self.assertEqual(set(archive.files), expected_keys)
                    self.assertEqual(
                        archive["records"].tobytes(), additions.tobytes()
                    )
            fine_xyz = np.column_stack(
                tuple(additions_by_resolution["1mm"][name] for name in ("x", "y", "z"))
            )
            coarse_xyz = np.column_stack(
                tuple(additions_by_resolution["5mm"][name] for name in ("x", "y", "z"))
            )
            fine_keys = {tuple(row) for row in fine_xyz.tolist()}
            self.assertTrue(all(tuple(row) in fine_keys for row in coarse_xyz.tolist()))
            cross_scale = manifest["cross_scale"]
            self.assertEqual(
                cross_scale["method"],
                "deterministic-maximal-surface-aware-fine-xyz-subset-v1",
            )
            self.assertTrue(
                cross_scale["invariants"]["coarse_addition_xyz_is_exact_fine_subset"]
            )
            self.assertEqual(
                cross_scale["invariants"]["coarse_independent_geometry_proposals"],
                0,
            )
            for role in ("SAND", "ROCK"):
                role_cross_scale = cross_scale["roles"][role]
                self.assertTrue(role_cross_scale["coverage_verified"])
                self.assertTrue(role_cross_scale["terrain_clearance_verified"])
                self.assertTrue(
                    role_cross_scale["vertical_support_guard_verified"]
                )
                self.assertTrue(role_cross_scale["exact_xyz_subset_verified"])
            for path, expected_hash in source_hashes.items():
                self.assertEqual(PIPE.sha256_path(path), expected_hash)
            with self.assertRaisesRegex(FileExistsError, "overwrite"):
                PIPE.build_terrain_candidates(
                    sand_1mm_path=sand_1mm,
                    rock_1mm_base_path=rock_1mm,
                    sand_5mm_path=sand_5mm,
                    rock_5mm_base_path=rock_5mm,
                    config_path=config,
                    cleanmesh_executable=executable,
                    output_dir=destination,
                    normalization_manifest_path=normalization_manifest,
                    parameters=parameters,
                    cleanmesh_runner=fake_cleanmesh_runner,
                    require_scipy=False,
                )

    def test_coarse_build_rejects_mutated_fine_authority(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            holes = {(2, 2), (3, 2), (2, 3), (3, 3)}
            sand_1mm = root / "sand-1mm.ply"
            rock_1mm = root / "rock-1mm.candidate.ply"
            sand_5mm = root / "sand-5mm.ply"
            rock_5mm = root / "rock-5mm.candidate.ply"
            write_ply(sand_1mm, plane_records(holes=holes))
            write_ply(rock_1mm, plane_records(origin=(1.0, 1.0)))
            write_ply(sand_5mm, plane_records(holes=holes))
            write_ply(rock_5mm, plane_records(origin=(1.0, 1.0)))
            executable = root / "fake-cleanmesh"
            executable.write_bytes(b"fake cleanmesh identity\n")
            executable.chmod(0o755)
            parameters = PIPE.PipelineParameters(
                fine=resolution("1mm", 0.005),
                coarse=resolution("5mm", 0.010),
                role_dominance_ratio=1.0,
                chunk_records=19,
                cleanmesh_chunk_points=100,
                cleanmesh_normalization_samples=100,
            )
            fine = PIPE.build_resolution_candidates(
                sand_path=sand_1mm,
                rock_base_path=rock_1mm,
                targets=(target(),),
                output_dir=root / "fine",
                cleanmesh_executable=executable,
                resolution=parameters.fine,
                pipeline=parameters,
                cleanmesh_runner=fake_cleanmesh_runner,
            )
            archive_path = Path(fine.addition_archive_paths["SAND"])
            with archive_path.open("r+b") as handle:
                handle.seek(-1, os.SEEK_END)
                original = handle.read(1)
                handle.seek(-1, os.SEEK_END)
                handle.write(bytes([original[0] ^ 0x01]))
            with self.assertRaisesRegex(RuntimeError, "archive hash changed"):
                PIPE.build_cross_scale_coarse_candidates(
                    sand_path=sand_5mm,
                    rock_base_path=rock_5mm,
                    fine=fine,
                    targets=(target(),),
                    output_dir=root / "coarse",
                    cleanmesh_executable=executable,
                    resolution=parameters.coarse,
                    pipeline=parameters,
                    cleanmesh_runner=fake_cleanmesh_runner,
                )

    def test_coarse_build_fails_without_vertical_same_role_donor(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            holes = {(2, 2), (3, 2), (2, 3), (3, 3)}
            sand_1mm = root / "sand-1mm.ply"
            rock_1mm = root / "rock-1mm.candidate.ply"
            sand_5mm = root / "sand-5mm.ply"
            rock_5mm = root / "rock-5mm.candidate.ply"
            write_ply(sand_1mm, plane_records(holes=holes))
            write_ply(rock_1mm, plane_records(origin=(1.0, 1.0)))
            incompatible = plane_records(holes=holes)
            incompatible["z"] += 0.050
            write_ply(sand_5mm, incompatible)
            write_ply(rock_5mm, plane_records(origin=(1.0, 1.0)))
            executable = root / "fake-cleanmesh"
            executable.write_bytes(b"fake cleanmesh identity\n")
            executable.chmod(0o755)
            parameters = PIPE.PipelineParameters(
                fine=resolution("1mm", 0.005),
                coarse=resolution("5mm", 0.010),
                role_dominance_ratio=1.0,
                chunk_records=19,
                cleanmesh_chunk_points=100,
                cleanmesh_normalization_samples=100,
                cross_scale_vertical_tolerance_m=0.012,
            )
            fine = PIPE.build_resolution_candidates(
                sand_path=sand_1mm,
                rock_base_path=rock_1mm,
                targets=(target(),),
                output_dir=root / "fine",
                cleanmesh_executable=executable,
                resolution=parameters.fine,
                pipeline=parameters,
                cleanmesh_runner=fake_cleanmesh_runner,
            )
            with self.assertRaisesRegex(
                RuntimeError, "fine points cannot be covered"
            ):
                PIPE.build_cross_scale_coarse_candidates(
                    sand_path=sand_5mm,
                    rock_base_path=rock_5mm,
                    fine=fine,
                    targets=(target(),),
                    output_dir=root / "coarse",
                    cleanmesh_executable=executable,
                    resolution=parameters.coarse,
                    pipeline=parameters,
                    cleanmesh_runner=fake_cleanmesh_runner,
                )

    def test_source_hash_lock_detects_same_size_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "cloud.ply"
            write_ply(path, plane_records(width_cells=1, height_cells=1))
            fingerprint = PIPE.fingerprint_ply(path)
            stat = path.stat()
            with path.open("r+b") as handle:
                handle.seek(-1, os.SEEK_END)
                value = handle.read(1)
                handle.seek(-1, os.SEEK_END)
                handle.write(bytes([value[0] ^ 0x01]))
            os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            with self.assertRaisesRegex(RuntimeError, "hash changed"):
                PIPE.assert_fingerprint_unchanged(fingerprint)


if __name__ == "__main__":
    unittest.main()
