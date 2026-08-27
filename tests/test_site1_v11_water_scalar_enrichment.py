from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "site1_v11_water_scalar_enrichment.py"
SPEC = importlib.util.spec_from_file_location(
    "site1_v11_water_scalar_enrichment", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
ENRICH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ENRICH
SPEC.loader.exec_module(ENRICH)


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


DERIVED_FIELDS = [
    (f"scalar_A_R_{metric}_{scale}", "<f4")
    for metric in ENRICH.PHYSICAL_METRICS
    for scale in ENRICH.DERIVED_SCALES
]
DERIVED_FIELDS += [
    (f"scalar_A_R_{metric}_Combined", "<f4")
    for metric in ENRICH.PHYSICAL_METRICS
]
DERIVED_FIELDS += [("scalar_A_R_RoughnessRelative_FineMedium", "<f4")]
DERIVED_FIELDS += [(name, "<f4") for name in ENRICH.VISIBILITY_FIELDS]
DERIVED_FIELDS += [
    ("scalar_A_R_Downhill_X", "<f4"),
    ("scalar_A_R_Downhill_Y", "<f4"),
    ("scalar_A_R_Downhill_Z", "<f4"),
    ("scalar_A_R_DownhillMagnitude", "<f4"),
    ("scalar_A_R_Horizontalness", "<f4"),
    ("scalar_A_R_Slope_deg", "<f4"),
]


FULL_DTYPE = np.dtype(
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
        *DERIVED_FIELDS,
    ],
    align=False,
)


NORMALIZATIONS = {
    metric: {"Fine": 2.0, "Medium": 4.0, "Broad": 8.0}
    for metric in ENRICH.PHYSICAL_METRICS
}


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
    layout = ENRICH.terrain.inspect_fixed_stride_ply(path)
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


def records(xyz, *, scan_id: float, start: int = 0) -> np.ndarray:
    xyz = np.asarray(xyz, np.float64)
    output = np.zeros(len(xyz), FULL_DTYPE)
    output["x"], output["y"], output["z"] = xyz.T
    output["red"] = 100 + start
    output["green"] = 110 + start
    output["blue"] = 120 + start
    output["nz"] = 1.0
    output["scalar_Intensity"] = 6_000.0 + start + np.arange(len(xyz))
    output["scalar_Composite"] = 90.0 + start
    output["scalar_ScanID"] = scan_id
    for name, _ in DERIVED_FIELDS:
        output[name] = 99.0
    for name in ENRICH.VISIBILITY_FIELDS:
        output[name] = 0.30 + 0.01 * start
    return output


def fake_cleanmesh_runner(
    executable,
    input_path,
    output_path,
    report_path,
    resolution,
    pipeline,
):
    del executable, resolution, pipeline
    source = read_ply(input_path)
    missing = [field for field, _ in DERIVED_FIELDS if field not in source.dtype.names]
    analysed_dtype = np.dtype(
        list(source.dtype.descr)
        + [(name, FULL_DTYPE[name]) for name in missing],
        align=False,
    )
    analysed = np.empty(len(source), analysed_dtype)
    for name in source.dtype.names or ():
        analysed[name] = source[name]
    for name in missing:
        analysed[name] = 0.0
    for metric in ENRICH.PHYSICAL_METRICS:
        analysed[f"scalar_A_R_{metric}_Fine"] = 1.0
        analysed[f"scalar_A_R_{metric}_Medium"] = 2.0
        analysed[f"scalar_A_R_{metric}_Broad"] = 4.0
        analysed[f"scalar_A_R_{metric}_Combined"] = -77.0
    analysed["scalar_A_R_RoughnessRelative_FineMedium"] = -9.0
    for name in ENRICH.VISIBILITY_FIELDS:
        analysed[name] = 0.99
    for name in (
        "scalar_A_R_Downhill_X",
        "scalar_A_R_Downhill_Y",
        "scalar_A_R_Downhill_Z",
        "scalar_A_R_DownhillMagnitude",
        "scalar_A_R_Horizontalness",
        "scalar_A_R_Slope_deg",
    ):
        analysed[name] = 7.0
    # Deliberately emulate tiled reordering.  Extraction must use ScanID10 and
    # exact XYZ identity, not assume that the appended span remains contiguous.
    write_ply(output_path, analysed[::-1])
    report_path.write_text(json.dumps({"success": True}), encoding="utf-8")
    return {"runner": "fake-reordered", "input_points": int(len(source))}


def partially_undefined_cleanmesh_runner(
    executable,
    input_path,
    output_path,
    report_path,
    resolution,
    pipeline,
):
    audit = fake_cleanmesh_runner(
        executable,
        input_path,
        output_path,
        report_path,
        resolution,
        pipeline,
    )
    analysed = read_ply(output_path)
    tagged = np.flatnonzero(analysed["scalar_ScanID"] == 10.0)
    if len(tagged) < 2:
        raise AssertionError("partial fake requires two tagged additions")
    dependent = {
        *(f"scalar_A_R_{metric}_Combined" for metric in ENRICH.PHYSICAL_METRICS),
        "scalar_A_R_RoughnessRelative_FineMedium",
    }
    for name in ENRICH._geometry_fields(analysed.dtype):
        if name not in dependent:
            analysed[name][tagged[0]] = np.nan
    write_ply(output_path, analysed)
    return {**dict(audit), "one_tagged_addition_made_undefined": True}


class WaterScalarEnrichmentTests(unittest.TestCase):
    def fixture(self, root: Path):
        base = root / "water-base.candidate.ply"
        base_records = records(
            [
                (0.00, 0.00, 1.0),
                (0.10, 0.00, 1.0),
                (0.00, 0.10, 1.0),
                (0.10, 0.10, 1.0),
            ],
            scan_id=999.0,
        )
        write_ply(base, base_records)
        additions = records(
            [(0.035, 0.045, 1.001), (0.065, 0.055, 0.999)],
            scan_id=999.0,
            start=5,
        )
        geometry = root / "water-geometry.candidate.ply"
        ENRICH.hole_pipeline.append_candidate_records(base, additions, geometry)
        archive = root / "added-records.npz"
        np.savez_compressed(
            archive,
            records=additions,
            candidate_xy=np.column_stack(
                (additions["x"], additions["y"])
            ).astype(np.float64),
            candidate_label=np.ones(len(additions), np.int32),
        )
        manifest = root / "geometry-manifest.json"
        manifest.write_text(
            json.dumps(
                {
                    "source": {
                        "path": str(base),
                        "sha256": ENRICH.sha256_path(base),
                        "points": len(base_records),
                    },
                    "candidate": {
                        "path": str(geometry),
                        "sha256": ENRICH.sha256_path(geometry),
                        "points": len(base_records) + len(additions),
                    },
                    "archive": str(archive),
                    "archive_sha256": ENRICH.sha256_path(archive),
                    "addition_count": len(additions),
                    "holes": [
                        {
                            "seed_id": "test-hole",
                            "label": 1,
                            "accepted": True,
                            "bounds": [0.03, 0.07, 0.04, 0.06],
                        }
                    ],
                    "component_membership": {
                        "archive_key": "candidate_label",
                        "all_additions_assigned_to_accepted_component": True,
                        "accepted_labels": [1],
                    },
                    "existing_payload_byte_exact": True,
                }
            ),
            encoding="utf-8",
        )
        sand = root / "sand.ply"
        rock = root / "rock.ply"
        write_ply(
            sand,
            records(
                [(0.02, 0.04, 0.99), (0.08, 0.06, 1.01)],
                scan_id=3.0,
                start=10,
            ),
        )
        write_ply(
            rock,
            records(
                [(0.03, 0.03, 0.98), (0.07, 0.07, 1.02)],
                scan_id=4.0,
                start=20,
            ),
        )
        normalization = root / "normalization.json"
        normalization.write_text(
            json.dumps({"rock_combined_normalizations": NORMALIZATIONS}),
            encoding="utf-8",
        )
        executable = root / "cleanmesh"
        executable.write_bytes(b"fake-cleanmesh-test-binary")
        return {
            "base": base,
            "base_records": base_records,
            "additions": additions,
            "geometry": geometry,
            "archive": archive,
            "manifest": manifest,
            "sand": sand,
            "rock": rock,
            "normalization": normalization,
            "executable": executable,
        }

    def test_enrichment_replaces_stale_ar_and_preserves_donor_properties(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = self.fixture(root)
            output = root / "result"
            result = ENRICH.enrich_water_addition_scalars(
                base_water_path=fixture["base"],
                geometry_candidate_path=fixture["geometry"],
                geometry_manifest_path=fixture["manifest"],
                geometry_archive_path=fixture["archive"],
                sand_path=fixture["sand"],
                rock_path=fixture["rock"],
                cleanmesh_executable=fixture["executable"],
                normalization_manifest_path=fixture["normalization"],
                output_dir=output,
                resolution_label="2mm",
                nominal_spacing_m=0.002,
                collar_m=0.12,
                chunk_records=2,
                cleanmesh_runner=fake_cleanmesh_runner,
                require_scipy=False,
            )
            candidate = read_ply(result.candidate_path)
            base_count = len(fixture["base_records"])
            self.assertEqual(
                candidate[:base_count].tobytes(),
                fixture["base_records"].tobytes(),
            )
            suffix = candidate[base_count:]
            additions = fixture["additions"]
            np.testing.assert_array_equal(
                suffix["scalar_Intensity"], additions["scalar_Intensity"]
            )
            np.testing.assert_array_equal(
                suffix["scalar_Composite"], additions["scalar_Composite"]
            )
            np.testing.assert_array_equal(suffix["red"], additions["red"])
            np.testing.assert_array_equal(suffix["x"], additions["x"])
            np.testing.assert_array_equal(suffix["z"], additions["z"])
            self.assertTrue(np.all(suffix["scalar_ScanID"] == 999.0))
            self.assertTrue(
                np.all(suffix["scalar_A_R_MeanCurvature_Fine"] == 1.0)
            )
            self.assertTrue(
                np.all(suffix["scalar_A_R_MeanCurvature_Combined"] == 0.5)
            )
            self.assertTrue(
                np.all(suffix["scalar_A_R_RoughnessRelative_FineMedium"] == 0.5)
            )
            for name in ENRICH.VISIBILITY_FIELDS:
                np.testing.assert_array_equal(suffix[name], additions[name])
            manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
            self.assertTrue(
                manifest["invariants"]["geometry_metrics_from_local_cleanmesh"]
            )
            self.assertTrue(
                manifest["scalar_enrichment"]
                ["intensity_and_composition_archive_exact"]
            )
            self.assertEqual(
                manifest["local_analysis"]["tagged_addition_count"], 2
            )
            self.assertTrue(
                manifest["invariants"]
                ["geometry_component_membership_verified"]
            )
            self.assertEqual(
                manifest["scalar_enrichment"]
                ["geometry_component_membership"]
                ["component_label_count"],
                2,
            )
            self.assertEqual(
                set(manifest["scalar_enrichment_implementation"]),
                {
                    Path(ENRICH.__file__).name,
                    Path(ENRICH.terrain.__file__).name,
                    Path(ENRICH.terrain_pipeline.__file__).name,
                    Path(ENRICH.hole_pipeline.__file__).name,
                    Path(ENRICH.density.__file__).name,
                    Path(ENRICH.terrain_pipeline.confidence.__file__).name,
                },
            )
            self.assertEqual(
                manifest["parameters"]["semantic"],
                {
                    "resolution_label": "2mm",
                    "nominal_spacing_m": 0.002,
                    "water_analysis_type_id": 1,
                    "temporary_addition_scan_id": 10,
                    "final_water_scan_id": 999.0,
                    "local_collar_m": 0.12,
                    "minimum_combined_finite_fraction": 1.0,
                    "minimum_component_field_finite_fraction": 1.0,
                    "undefined_geometry_fallback": {
                        "method": "component-strict-multiscale-3d-idw",
                        "radii_m": [0.02, 0.04, 0.08, 0.12, 0.24, 0.48, 0.96, 8.0],
                        "component_diameter_bound_m": 8.0,
                        "maximum_neighbours": 16,
                        "minimum_neighbours": 3,
                        "tiny_component_max_points": 64,
                        "cross_component_borrowing": False,
                        "imputed_values_may_become_donors": False,
                        "only_original_finite_values_may_be_donors": True,
                        "multi_hop_propagation": False,
                        "local_donor_range_bounded": True,
                    },
                    "coarse_geometry_source": "local-cleanmesh",
                },
            )
            self.assertEqual(
                manifest["parameters"]["execution_only"][
                    "outer_io_chunk_records"
                ],
                2,
            )
            self.assertEqual(
                manifest["parameters"]["cleanmesh_invocation"][
                    "fields_read_by_production_runner"
                ],
                [
                    "resolution.cleanmesh_base_voxel_m",
                    "pipeline.cleanmesh_tile_width_m",
                    "pipeline.cleanmesh_chunk_points",
                    "pipeline.cleanmesh_normalization_samples",
                ],
            )

    def test_component_strict_fallback_fills_only_nan_and_rebuilds_combined(self):
        initial = records(
            [
                (0.0000, 0.0, 1.0),
                (0.0100, 0.0, 1.0),
                (0.0200, 0.0, 1.0),
                (0.0300, 0.0, 1.0),
                (0.0001, 0.0, 1.0),
                (0.0101, 0.0, 1.0),
                (0.0201, 0.0, 1.0),
                (0.0301, 0.0, 1.0),
                (0.0900, 0.0, 1.0),
                (0.0950, 0.0, 1.0),
            ],
            scan_id=10.0,
        )
        analysed = initial.copy()
        labels = np.asarray([1, 1, 1, 1, 2, 2, 2, 2, 3, 3], np.int32)
        geometry_fields = ENRICH._geometry_fields(FULL_DTYPE)
        dependent = {
            *(f"scalar_A_R_{metric}_Combined" for metric in ENRICH.PHYSICAL_METRICS),
            "scalar_A_R_RoughnessRelative_FineMedium",
        }
        fallback_fields = [name for name in geometry_fields if name not in dependent]
        for name in fallback_fields:
            analysed[name][labels == 1] = 2.0
            analysed[name][labels == 2] = 20.0
            analysed[name][labels == 3] = 200.0
            analysed[name][[3, 7, 9]] = np.nan
        finite_before = {
            name: np.asarray(analysed[name][np.isfinite(analysed[name])]).tobytes()
            for name in fallback_fields
        }

        merged, audit = ENRICH.merge_analysed_geometry(
            analysed,
            initial,
            NORMALIZATIONS,
            component_labels=labels,
        )

        # Intermingled labels prove that the spatially closer other component
        # never donates.  The tiny two-row component uses its sole finite row.
        np.testing.assert_array_equal(
            merged["scalar_A_R_MeanCurvature_Fine"][[3, 7, 9]],
            np.asarray([2.0, 20.0, 200.0], np.float32),
        )
        for name in fallback_fields:
            self.assertEqual(
                np.asarray(merged[name][np.isfinite(analysed[name])]).tobytes(),
                finite_before[name],
            )
        np.testing.assert_array_equal(
            merged["scalar_Intensity"], initial["scalar_Intensity"]
        )
        np.testing.assert_array_equal(
            merged["scalar_Composite"], initial["scalar_Composite"]
        )
        self.assertAlmostEqual(
            float(merged["scalar_A_R_MeanCurvature_Combined"][3]), 0.675
        )
        self.assertEqual(
            float(merged["scalar_A_R_MeanCurvature_Combined"][7]), 1.0
        )
        before = audit["combined_field_finite_coverage_before_fallback"]
        after = audit["combined_field_finite_coverage"]
        self.assertEqual(
            before["scalar_A_R_MeanCurvature_Combined"]["finite"], 7
        )
        self.assertEqual(
            after["scalar_A_R_MeanCurvature_Combined"]["finite"], 10
        )
        fallback = audit["undefined_geometry_fallback"]
        self.assertFalse(fallback["cross_component_borrowing"])
        self.assertFalse(fallback["imputed_values_may_become_donors"])
        field = fallback["fields"]["scalar_A_R_MeanCurvature_Fine"]
        self.assertEqual(field["filled"], 3)
        self.assertEqual(field["unresolved"], 0)
        self.assertEqual(field["tiny_component_minimum_relaxed_labels"], [3])
        self.assertTrue(field["original_finite_values_byte_exact"])
        self.assertTrue(field["local_donor_range_bounded"])

    def test_end_to_end_fallback_uses_verified_archive_component_labels(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = self.fixture(root)
            result = ENRICH.enrich_water_addition_scalars(
                base_water_path=fixture["base"],
                geometry_candidate_path=fixture["geometry"],
                geometry_manifest_path=fixture["manifest"],
                geometry_archive_path=fixture["archive"],
                sand_path=fixture["sand"],
                rock_path=fixture["rock"],
                cleanmesh_executable=fixture["executable"],
                normalization_manifest_path=fixture["normalization"],
                output_dir=root / "partial-result",
                resolution_label="2mm",
                nominal_spacing_m=0.002,
                collar_m=0.12,
                chunk_records=2,
                cleanmesh_runner=partially_undefined_cleanmesh_runner,
                require_scipy=False,
            )
            candidate = read_ply(result.candidate_path)
            suffix = candidate[len(fixture["base_records"]) :]
            self.assertTrue(
                np.all(np.isfinite(suffix["scalar_A_R_MeanCurvature_Fine"]))
            )
            np.testing.assert_array_equal(
                suffix["scalar_Intensity"], fixture["additions"]["scalar_Intensity"]
            )
            manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
            fallback = manifest["scalar_enrichment"]["undefined_geometry_fallback"]
            self.assertEqual(fallback["component_labels"], [1])
            self.assertFalse(fallback["cross_component_borrowing"])
            field = fallback["fields"]["scalar_A_R_MeanCurvature_Fine"]
            self.assertEqual(field["undefined_before"], 1)
            self.assertEqual(field["filled"], 1)
            self.assertEqual(field["unresolved"], 0)
            self.assertEqual(field["tiny_component_minimum_relaxed_labels"], [1])
            self.assertTrue(
                manifest["invariants"]
                ["undefined_geometry_fallback_component_strict"]
            )
            self.assertTrue(
                manifest["invariants"]
                ["undefined_geometry_fallback_no_extrapolation"]
            )

    def test_component_strict_fallback_fails_if_component_has_no_finite_support(self):
        initial = records(
            [(0.01 * row, 0.0, 1.0) for row in range(10)],
            scan_id=10.0,
        )
        analysed = initial.copy()
        labels = np.asarray([1] * 8 + [2] * 2, np.int32)
        for name in ENRICH._geometry_fields(FULL_DTYPE):
            analysed[name] = 2.0
        for metric in ENRICH.PHYSICAL_METRICS:
            for scale in ENRICH.DERIVED_SCALES:
                analysed[f"scalar_A_R_{metric}_{scale}"][labels == 2] = np.nan
        with self.assertRaisesRegex(RuntimeError, "insufficient finite"):
            ENRICH.merge_analysed_geometry(
                analysed,
                initial,
                NORMALIZATIONS,
                component_labels=labels,
            )

    def test_sparse_fade_uses_larger_same_component_original_donors(self):
        initial = records(
            [
                (-1.00, 0.0, 1.0),
                (-0.99, 0.0, 1.0),
                (-0.98, 0.0, 1.0),
                (0.00, 0.0, 1.0),
                (0.01, 0.0, 1.0),
                (0.02, 0.0, 1.0),
                (0.30, 0.0, 1.0),
                (0.31, 0.0, 1.0),
            ],
            scan_id=10.0,
        )
        analysed = initial.copy()
        labels = np.asarray([1, 1, 1, 9, 9, 9, 9, 9], np.int32)
        geometry_fields = ENRICH._geometry_fields(FULL_DTYPE)
        dependent = {
            *(f"scalar_A_R_{metric}_Combined" for metric in ENRICH.PHYSICAL_METRICS),
            "scalar_A_R_RoughnessRelative_FineMedium",
        }
        fallback_fields = [name for name in geometry_fields if name not in dependent]
        for name in fallback_fields:
            analysed[name] = 2.0
            analysed[name][3:6] = [10.0, 12.0, 14.0]
            analysed[name][6:] = np.nan
        original = {
            name: np.asarray(analysed[name][3:6]).tobytes()
            for name in fallback_fields
        }
        merged, audit = ENRICH.merge_analysed_geometry(
            analysed,
            initial,
            NORMALIZATIONS,
            component_labels=labels,
        )
        for name in fallback_fields:
            self.assertTrue(np.all(np.isfinite(merged[name])))
            self.assertEqual(np.asarray(merged[name][3:6]).tobytes(), original[name])
            self.assertTrue(np.all(merged[name][6:] >= 10.0))
            self.assertTrue(np.all(merged[name][6:] <= 14.0))
        fallback = audit["undefined_geometry_fallback"]
        self.assertTrue(fallback["adaptive_larger_radius_tiers"])
        self.assertTrue(fallback["only_original_finite_values_may_be_donors"])
        field = fallback["fields"]["scalar_A_R_MeanCurvature_Fine"]
        self.assertEqual(
            field["original_finite_donor_count_by_component_label"]["9"], 3
        )
        self.assertEqual(field["filled_by_radius_m_by_component_label"]["9"]["0.48"], 2)
        self.assertEqual(field["unresolved_by_component_label"]["9"], 0)
        coverage = audit["component_field_finite_coverage"]
        self.assertTrue(coverage["all_components_all_required_fields_accepted"])

    def test_small_component_fails_even_when_global_coverage_exceeds_ninety_percent(self):
        initial = records(
            [(0.002 * row, 0.0, 1.0) for row in range(100)],
            scan_id=10.0,
        )
        analysed = initial.copy()
        labels = np.asarray([1] * 95 + [9] * 5, np.int32)
        for name in ENRICH._geometry_fields(FULL_DTYPE):
            analysed[name] = 2.0
        for metric in ENRICH.PHYSICAL_METRICS:
            for scale in ENRICH.DERIVED_SCALES:
                analysed[f"scalar_A_R_{metric}_{scale}"][labels == 9] = np.nan
        with self.assertRaisesRegex(
            RuntimeError,
            "insufficient finite per-component scalar coverage: label 9",
        ):
            ENRICH.merge_analysed_geometry(
                analysed,
                initial,
                NORMALIZATIONS,
                component_labels=labels,
            )

    def test_contract_rejects_suffix_that_no_longer_matches_archive(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = self.fixture(root)
            geometry_layout = ENRICH.terrain.inspect_fixed_stride_ply(
                fixture["geometry"]
            )
            memory = np.memmap(
                fixture["geometry"],
                dtype=geometry_layout.dtype,
                mode="r+",
                offset=geometry_layout.offset,
                shape=(geometry_layout.vertex_count,),
            )
            memory[-1]["scalar_Intensity"] += 1.0
            memory.flush()
            del memory
            document = json.loads(fixture["manifest"].read_text(encoding="utf-8"))
            document["candidate"]["sha256"] = ENRICH.sha256_path(
                fixture["geometry"]
            )
            fixture["manifest"].write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "suffix differs"):
                ENRICH.verify_append_only_geometry(
                    base_water_path=fixture["base"],
                    geometry_candidate_path=fixture["geometry"],
                    geometry_manifest_path=fixture["manifest"],
                    geometry_archive_path=fixture["archive"],
                    chunk_records=2,
                )

    def test_contract_replays_reversible_base_cull_and_preserves_survivors(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base_records = records(
                [(0.02 * row, 0.0, 1.0) for row in range(6)],
                scan_id=999.0,
            )
            removed_indices = np.asarray([1, 4], dtype="<i8")
            keep = np.ones(len(base_records), dtype=bool)
            keep[removed_indices] = False
            additions = records(
                [(0.03, 0.03, 1.001), (0.07, 0.03, 0.999)],
                scan_id=999.0,
                start=20,
            )
            base = root / "base.candidate.ply"
            geometry = root / "geometry.candidate.ply"
            write_ply(base, base_records)
            write_ply(
                geometry,
                np.concatenate((base_records[keep], additions)),
            )
            addition_archive = root / "additions.npz"
            np.savez(addition_archive, records=additions)
            record_archive = root / "far-lobe-cull.records.bin"
            index_archive = root / "far-lobe-cull.source-indices.i64"
            base_records[removed_indices].tofile(record_archive)
            removed_indices.tofile(index_archive)

            def fingerprint(path: Path, *, points=None):
                stat = path.stat()
                result = {
                    "path": str(path.resolve()),
                    "size_bytes": stat.st_size,
                    "mtime_ns": stat.st_mtime_ns,
                    "sha256": ENRICH.sha256_path(path),
                }
                if points is not None:
                    result["points"] = points
                return result

            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({
                "source": {
                    "path": str(base),
                    "sha256": ENRICH.sha256_path(base),
                    "points": len(base_records),
                },
                "candidate": {
                    "path": str(geometry),
                    "sha256": ENRICH.sha256_path(geometry),
                    "points": len(base_records) - 2 + len(additions),
                },
                "archive": str(addition_archive),
                "archive_sha256": ENRICH.sha256_path(addition_archive),
                "addition_count": len(additions),
                "far_lobe_cull": {
                    "performed": True,
                    "reversible": True,
                    "measured_no_eligible_component": False,
                    "removed_count": 2,
                    "archive": {
                        "removed_count": 2,
                        "record_stride_bytes": base_records.dtype.itemsize,
                        "source_index_dtype": "<i8",
                        "exact_source_rows_archived": True,
                        "exact_source_indices_archived": True,
                        "records": fingerprint(record_archive),
                        "source_indices": fingerprint(index_archive, points=2),
                    },
                },
            }), encoding="utf-8")
            contract, observed_additions = ENRICH.verify_append_only_geometry(
                base_water_path=base,
                geometry_candidate_path=geometry,
                geometry_manifest_path=manifest,
                geometry_archive_path=addition_archive,
                chunk_records=2,
            )
            self.assertEqual(contract.source_points, 6)
            self.assertEqual(contract.removed_base_count, 2)
            self.assertEqual(contract.base_points, 4)
            self.assertEqual(contract.addition_count, 2)
            np.testing.assert_array_equal(observed_additions, additions)

            enriched = additions.copy()
            enriched["scalar_A_R_MeanCurvature_Fine"] = [2.0, 3.0]
            output = root / "enriched.candidate.ply"
            ENRICH._write_enriched_geometry_candidate(
                geometry,
                enriched,
                output,
                contract,
                chunk_records=2,
            )
            observed = read_ply(output)
            self.assertEqual(
                observed[:4].tobytes(), base_records[keep].tobytes()
            )
            self.assertEqual(observed[4:].tobytes(), enriched.tobytes())

    def test_5mm_uses_exact_fine_mapping_and_keeps_cleanmesh_diagnostic_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = self.fixture(root)
            coarse = fixture["additions"]
            fine_additions = records(
                [
                    (coarse[1]["x"], coarse[1]["y"], coarse[1]["z"]),
                    (0.050, 0.050, 1.000),
                    (coarse[0]["x"], coarse[0]["y"], coarse[0]["z"]),
                ],
                scan_id=999.0,
                start=30,
            )
            for row in range(len(fine_additions)):
                for name in ENRICH._geometry_fields(FULL_DTYPE):
                    fine_additions[row][name] = 0.10 * (row + 1)
            fine_candidate = root / "fine-enriched.candidate.ply"
            ENRICH.hole_pipeline.append_candidate_records(
                fixture["base"], fine_additions, fine_candidate
            )
            fine_archive = root / "fine-added-records.npz"
            fine_candidate_xy = np.column_stack(
                (fine_additions["x"], fine_additions["y"])
            ).astype(np.float64)
            fine_candidate_label = np.full(len(fine_additions), 4, np.int32)
            np.savez_compressed(
                fine_archive,
                records=fine_additions,
                candidate_xy=fine_candidate_xy,
                candidate_label=fine_candidate_label,
            )
            fine_geometry_manifest = root / "fine-geometry-manifest.json"
            fine_geometry_manifest.write_text(
                json.dumps(
                    {
                        "candidate": {
                            "path": str(fine_candidate),
                            "sha256": ENRICH.sha256_path(fine_candidate),
                        },
                        "archive": str(fine_archive),
                        "archive_sha256": ENRICH.sha256_path(fine_archive),
                        "addition_count": len(fine_additions),
                    }
                ),
                encoding="utf-8",
            )
            fine_geometry_stat = fine_geometry_manifest.stat()
            fine_archive_stat = fine_archive.stat()
            fine_manifest = root / "fine-enriched-manifest.json"
            fine_manifest.write_text(
                json.dumps(
                    {
                        "operation": (
                            "site1-v11-candidate-only-water-addition-"
                            "scalar-enrichment"
                        ),
                        "resolution_label": "2mm",
                        "candidate": {
                            "path": str(fine_candidate),
                            "sha256": ENRICH.sha256_path(fine_candidate),
                            "points": len(fixture["base_records"])
                            + len(fine_additions),
                        },
                        "geometry_contract": {
                            "base_points": len(fixture["base_records"]),
                            "addition_count": len(fine_additions),
                            "candidate_points": len(fixture["base_records"])
                            + len(fine_additions),
                            "base_payload_sha256": ENRICH._payload_sha256(
                                ENRICH.terrain.inspect_fixed_stride_ply(
                                    fixture["base"]
                                )
                            ),
                            "candidate_sha256": ENRICH.sha256_path(
                                fine_candidate
                            ),
                        },
                        "geometry_manifest": {
                            "path": str(fine_geometry_manifest.resolve()),
                            "size_bytes": fine_geometry_stat.st_size,
                            "mtime_ns": fine_geometry_stat.st_mtime_ns,
                            "sha256": ENRICH.sha256_path(
                                fine_geometry_manifest
                            ),
                        },
                        "archive_sha256": ENRICH.sha256_path(fine_archive),
                        "input_fingerprints": {
                            "geometry_manifest": {
                                "path": str(fine_geometry_manifest.resolve()),
                                "size_bytes": fine_geometry_stat.st_size,
                                "mtime_ns": fine_geometry_stat.st_mtime_ns,
                                "sha256": ENRICH.sha256_path(
                                    fine_geometry_manifest
                                ),
                            },
                            "geometry_archive": {
                                "path": str(fine_archive.resolve()),
                                "size_bytes": fine_archive_stat.st_size,
                                "mtime_ns": fine_archive_stat.st_mtime_ns,
                                "sha256": ENRICH.sha256_path(fine_archive),
                            },
                            "normalization_manifest": {
                                "path": str(fixture["normalization"].resolve()),
                                "size_bytes": fixture["normalization"].stat().st_size,
                                "mtime_ns": fixture["normalization"].stat().st_mtime_ns,
                                "sha256": ENRICH.sha256_path(
                                    fixture["normalization"]
                                ),
                            },
                        },
                        "holes": [
                            {
                                "seed_id": "mark4",
                                "label": 4,
                                "accepted": True,
                                "bounds": [0.0, 0.1, 0.0, 0.1],
                            }
                        ],
                        "component_membership": {
                            "archive_key": "candidate_label",
                            "all_additions_assigned_to_accepted_component": True,
                            "accepted_labels": [4],
                        },
                        "invariants": {
                            "geometry_metrics_from_local_cleanmesh": True
                        },
                    }
                ),
                encoding="utf-8",
            )
            # The coarse archive is the authoritative order mapping into the
            # fine addition suffix.  It deliberately selects non-monotonic
            # rows to prove that coordinates, not positional coincidence, are
            # verified.
            np.savez_compressed(
                fixture["archive"],
                records=coarse,
                fine_selection_index=np.asarray([2, 0], np.int64),
                fine_component_label=np.asarray([4, 4], np.int32),
            )
            document = json.loads(fixture["manifest"].read_text(encoding="utf-8"))
            document["archive_sha256"] = ENRICH.sha256_path(fixture["archive"])
            document.pop("component_membership")
            document["holes"] = [
                {
                    "seed_id": "mark4",
                    "label": 4,
                    "accepted": True,
                    "bounds": [0.0, 0.1, 0.0, 0.1],
                }
            ]
            document["terrain_sources"] = {"sand": "hash-locked", "rock": "hash-locked"}
            document["cross_scale"] = {
                "method": "deterministic-variable-radius-blue-noise-subset-v1",
                "fine_manifest": {
                    "path": str(fine_geometry_manifest.resolve()),
                    "size_bytes": fine_geometry_stat.st_size,
                    "mtime_ns": fine_geometry_stat.st_mtime_ns,
                    "sha256": ENRICH.sha256_path(fine_geometry_manifest),
                },
                "fine_archive": {
                    "path": str(fine_archive.resolve()),
                    "size_bytes": fine_archive_stat.st_size,
                    "mtime_ns": fine_archive_stat.st_mtime_ns,
                    "sha256": ENRICH.sha256_path(fine_archive),
                },
                "fine_candidate_sha256": ENRICH.sha256_path(fine_candidate),
                "fine_addition_count": len(fine_additions),
                "fine_selection_index_count": len(coarse),
                "fine_selection_index_unique": True,
                "coarse_xyz_exact_subset_of_fine_records_xyz": True,
                "coarse_normals_exact_subset_of_fine_records_normals": True,
                "nongeometry_fields_preserved_from_coarse_donors": True,
                "geometry_fields_copied_from_fine_records": [
                    "x",
                    "y",
                    "z",
                    "nx",
                    "ny",
                    "nz",
                ],
                "selection_seed": 23,
                "spacing_m": 0.005,
                "maximum_fine_to_coarse_or_terrain_support_distance_m": 0.004,
                "accepted_hole_coverage": [
                    {
                        "seed_id": "mark4",
                        "component_label": 4,
                        "fine_count": len(fine_additions),
                        "coarse_addition_count": len(coarse),
                        "bounds": [0.0, 0.1, 0.0, 0.1],
                    }
                ],
            }
            fixture["manifest"].write_text(json.dumps(document), encoding="utf-8")
            output = root / "coarse-result"
            result = ENRICH.enrich_water_addition_scalars(
                base_water_path=fixture["base"],
                geometry_candidate_path=fixture["geometry"],
                geometry_manifest_path=fixture["manifest"],
                geometry_archive_path=fixture["archive"],
                sand_path=fixture["sand"],
                rock_path=fixture["rock"],
                cleanmesh_executable=fixture["executable"],
                normalization_manifest_path=fixture["normalization"],
                output_dir=output,
                resolution_label="5mm",
                nominal_spacing_m=0.005,
                fine_enriched_candidate_path=fine_candidate,
                fine_enriched_manifest_path=fine_manifest,
                collar_m=0.12,
                chunk_records=2,
                cleanmesh_runner=fake_cleanmesh_runner,
                require_scipy=False,
            )
            candidate = read_ply(result.candidate_path)
            suffix = candidate[len(fixture["base_records"]):]
            expected = fine_additions[[2, 0]]
            for name in ENRICH._geometry_fields(FULL_DTYPE):
                np.testing.assert_array_equal(suffix[name], expected[name])
            np.testing.assert_array_equal(
                suffix["scalar_Intensity"], coarse["scalar_Intensity"]
            )
            np.testing.assert_array_equal(
                suffix["scalar_Composite"], coarse["scalar_Composite"]
            )
            for name in ENRICH.VISIBILITY_FIELDS:
                np.testing.assert_array_equal(suffix[name], coarse[name])
            self.assertTrue(np.all(suffix["scalar_ScanID"] == 999.0))
            manifest = json.loads(result.manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["terrain_sources"], document["terrain_sources"])
            self.assertEqual(manifest["cross_scale"], document["cross_scale"])
            self.assertEqual(
                manifest["scalar_enrichment"]["fine_selection_index"]["count"],
                2,
            )
            self.assertFalse(
                manifest["local_analysis"]["output_policy"]["accepted_for_output"]
            )
            self.assertTrue(
                manifest["scalar_enrichment"]["cross_scale_verification"]
                ["fine_coarse_normalization_manifest_sha256_verified"]
            )
            self.assertTrue(
                manifest["scalar_enrichment"]["component_membership"]
                ["coarse_labels_match_selected_fine_labels"]
            )
            self.assertTrue(
                manifest["invariants"]
                ["coarse_geometry_metrics_from_exact_fine_selection"]
            )
            np.savez_compressed(
                fixture["archive"],
                records=coarse,
                fine_selection_index=np.asarray([2, 0], np.int64),
                fine_component_label=np.asarray([4, 5], np.int32),
            )
            document["archive_sha256"] = ENRICH.sha256_path(fixture["archive"])
            fixture["manifest"].write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(
                RuntimeError,
                "fine_component_label differs from selected fine labels",
            ):
                ENRICH.enrich_water_addition_scalars(
                    base_water_path=fixture["base"],
                    geometry_candidate_path=fixture["geometry"],
                    geometry_manifest_path=fixture["manifest"],
                    geometry_archive_path=fixture["archive"],
                    sand_path=fixture["sand"],
                    rock_path=fixture["rock"],
                    cleanmesh_executable=fixture["executable"],
                    normalization_manifest_path=fixture["normalization"],
                    output_dir=root / "bad-component-result",
                    resolution_label="5mm",
                    nominal_spacing_m=0.005,
                    fine_enriched_candidate_path=fine_candidate,
                    fine_enriched_manifest_path=fine_manifest,
                    collar_m=0.12,
                    chunk_records=2,
                    cleanmesh_runner=fake_cleanmesh_runner,
                    require_scipy=False,
                )

    def test_collar_selection_excludes_scanid9_terrain_and_far_points(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            additions = records([(0.0, 0.0, 1.0)], scan_id=999.0)
            source = records(
                [(0.01, 0.0, 1.0), (0.02, 0.0, 1.0), (2.0, 2.0, 1.0)],
                scan_id=1.0,
            )
            source[1]["scalar_ScanID"] = 9.0
            path = root / "terrain.ply"
            write_ply(path, source)
            _, selected, audit = ENRICH.collect_collar_indices(
                path,
                additions,
                collar_m=0.05,
                measured_only=True,
                chunk_records=2,
            )
            np.testing.assert_array_equal(selected, [0])
            self.assertTrue(audit["selection_is_local_collar_only"])


if __name__ == "__main__":
    unittest.main()
