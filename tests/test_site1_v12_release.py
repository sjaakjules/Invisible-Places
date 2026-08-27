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
    / "site1_v12_release.py"
)
SPEC = importlib.util.spec_from_file_location("site1_v12_release", SCRIPT)
V12 = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = V12
SPEC.loader.exec_module(V12)


DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
        ("scalar_ScanID", "<f4"),
        ("scalar_Intensity", "<f4"),
        ("scalar_A_R_MeanCurvature_Fine", "<f4"),
        ("scalar_A_R_Roughness_Combined", "<f4"),
        ("scalar_A_R_RoughnessRelative_FineMedium", "<f4"),
    ]
)


def records(count, *, start=0.0, scan_id=999.0):
    value = np.zeros(count, DTYPE)
    index = np.arange(count, dtype=np.float32)
    value["x"] = start + index * 0.0021
    value["y"] = 815.0 + index * 0.0017
    value["z"] = 2.0 + index * 0.00013
    value["nx"] = 0.01
    value["ny"] = -0.02
    value["nz"] = 0.99975
    value["red"] = np.arange(count, dtype=np.uint8) + 10
    value["green"] = np.arange(count, dtype=np.uint8) + 20
    value["blue"] = np.arange(count, dtype=np.uint8) + 30
    value["scalar_ScanID"] = scan_id
    value["scalar_Intensity"] = 5000.0 + index * 17.0
    value["scalar_A_R_MeanCurvature_Fine"] = 0.1 + index * 0.001
    value["scalar_A_R_Roughness_Combined"] = 0.2 + index * 0.001
    value["scalar_A_R_RoughnessRelative_FineMedium"] = 1.0 + index * 0.01
    return value


def write_ply(path, value, *, comment="fixture"):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    type_name = {
        "<f4": "float",
        "|u1": "uchar",
    }
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"comment {comment}",
        f"element vertex {len(value)}",
    ]
    for name in value.dtype.names:
        lines.append(f"property {type_name[value.dtype.fields[name][0].str]} {name}")
    lines.append("end_header")
    with path.open("xb") as handle:
        handle.write(("\n".join(lines) + "\n").encode("ascii"))
        handle.write(value.tobytes())
    return path


def write_report(path, fine, coarse, **changes):
    fine_info = V12.inspect_ply(fine)
    coarse_info = V12.inspect_ply(coarse)
    value = {
        "input": str(Path(fine).resolve()),
        "output": str(Path(coarse).resolve()),
        "method": "greedy_spatial_minimum_distance",
        "minimum_spacing_m": 0.005,
        "source_points": fine_info.count,
        "output_points": coarse_info.count,
        "record_stride": fine_info.stride,
        "occupied_cells": coarse_info.count,
        "non_finite_positions": 0,
    }
    value.update(changes)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return path


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return path


def upstream_fingerprint(path, *, points=None):
    source = Path(path).resolve()
    value = source.stat()
    result = {
        "path": str(source),
        "size_bytes": value.st_size,
        "mtime_ns": value.st_mtime_ns,
        "sha256": V12.sha256_path(source),
    }
    if points is not None:
        result["points"] = int(points)
    return result


def audit_fingerprint(path, *, ply=False):
    source = Path(path).resolve()
    value = source.stat()
    result = {
        "path": str(source),
        "size_bytes": value.st_size,
        "mtime_ns": value.st_mtime_ns,
        "sha256": V12.sha256_path(source),
    }
    if ply:
        info = V12.inspect_ply(source)
        result.update(
            points=info.count,
            record_stride=info.stride,
            schema=[list(item) for item in info.schema],
        )
    return result


def implementation_hashes(names):
    return {
        name: V12.sha256_path(V12.SCRIPT_DIR / name)
        for name in names
    }


class Fixture:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.data = self.root / "Data" / "Scene1"
        self.run = self.data / "PatchRefinement" / "v12"
        self.data.mkdir(parents=True)
        self.run.mkdir(parents=True)

        self.source_fine_records = records(5, start=100.0, scan_id=2.0)
        self.source_coarse_records = self.source_fine_records[[0, 2, 4]].copy()
        self.addition_records = records(8, start=770.0)
        self.fine_records = np.concatenate(
            (self.source_fine_records, self.addition_records)
        )
        self.coarse_records = self.fine_records[[0, 2, 5, 8, 12]].copy()
        self.canonical_fine = write_ply(
            self.data / "Site1-WATER-2mm.ply",
            self.source_fine_records,
            comment="current fine",
        )
        self.canonical_coarse = write_ply(
            self.data / "Site1-WATER-5mm.ply",
            self.source_coarse_records,
            comment="current coarse",
        )
        self.old01 = write_ply(
            self.data / "Site1-WATER-5mm-old01.ply",
            records(2, start=40.0, scan_id=1.0),
            comment="historic protected generation",
        )
        self.sand_sentinel = write_ply(
            self.data / "Site1-SAND-1mm.ply",
            records(3, start=720.0, scan_id=3.0),
            comment="terrain-must-not-change",
        )
        self.rock_sentinel = write_ply(
            self.data / "Site1-ROCK-1mm.ply",
            records(4, start=730.0, scan_id=4.0),
            comment="rock-must-not-change",
        )

        self.review_config = V12.DEFAULT_REVIEW_CONFIG
        self.normalization = write_json(
            self.root / "v11" / "normalization-manifest.json",
            {"operation": "fixture-normalization", "verified": True},
        )
        self.cleanmesh = self.root / "bin" / "cleanmesh_reduced_analysis"
        self.cleanmesh.parent.mkdir(parents=True)
        self.cleanmesh.write_bytes(b"fixture reduced analysis executable")
        self.downsample_executable = self.root / "bin" / "cleanmesh_spatial_downsample"
        self.downsample_executable.write_bytes(b"fixture downsample executable")

        self.geometry = write_ply(
            self.run
            / "water-geometry-2mm"
            / "Site1-WATER-2mm.geometry-v12.candidate.ply",
            self.fine_records,
            comment="base plus archived v12 geometry",
        )
        self.geometry_archive = self.run / "water-geometry-2mm" / "additions.npz"
        self.component_labels = np.ones(len(self.addition_records), np.int32)
        np.savez(
            self.geometry_archive,
            records=self.addition_records,
            candidate_label=self.component_labels,
        )
        geometry_document = {
            "version": 1,
            "algorithm": "site1-v12-fine-first-supported-water-interface-v1",
            "candidate_only": True,
            "canonical_install_performed": False,
            "existing_payload_byte_exact": True,
            "source": upstream_fingerprint(
                self.canonical_fine, points=len(self.source_fine_records)
            ),
            "candidate": upstream_fingerprint(
                self.geometry, points=len(self.fine_records)
            ),
            "config": upstream_fingerprint(self.review_config),
            "archive": str(self.geometry_archive.resolve()),
            "archive_sha256": V12.sha256_path(self.geometry_archive),
            "addition_count": len(self.addition_records),
            "parameters": {
                "fine_first": True,
                "coarse_recomputation_allowed": False,
            },
            "far_lobe_cull": {
                "performed": False,
                "reversible": True,
                "measured_no_eligible_component": True,
                "reason": "fixture found no eligible detached component",
                "seed_xy": [0.0, 0.0],
                "maximum_seed_distance_m": 2.0,
                "grid_pitch_m": 0.025,
                "bridge_radius_m": 0.05,
                "bridge_iterations": 3,
                "detachment_gap_m": 0.15,
                "maximum_component_fraction": 0.2,
                "occupied_cell_count": 5,
                "selected_occupied_cell_count": 0,
                "largest_occupied_cell_count": 5,
                "seed_distance_m": None,
                "component_fraction": None,
                "minimum_cell_center_separation_m": None,
                "minimum_point_separation_lower_bound_m": None,
                "removed_count": 0,
                "source_count_before": len(self.source_fine_records),
                "surviving_source_count": len(self.source_fine_records),
                "surviving_source_payload_byte_exact": True,
                "surviving_source_row_order_preserved": True,
                "archive": None,
            },
            "implementation_sha256": implementation_hashes(
                V12._GEOMETRY_IMPLEMENTATIONS
            ),
        }
        self.geometry_manifest = write_json(
            self.run / "water-geometry-2mm" / "manifest.json",
            geometry_document,
        )

        self.fine = write_ply(
            self.run / "water-final-2mm" / "Site1-WATER-2mm.candidate.ply",
            self.fine_records,
            comment="fine candidate has a distinct header",
        )
        self.coarse = write_ply(
            self.run / "water-final-5mm" / "Site1-WATER-5mm.candidate.ply",
            self.coarse_records,
            comment="coarse header may differ",
        )
        geometry_copy = self.run / "water-final-2mm" / "geometry-manifest.json"
        geometry_copy.write_bytes(self.geometry_manifest.read_bytes())
        base_info = V12.inspect_ply(self.canonical_fine)
        geometry_info = V12.inspect_ply(self.geometry)
        fine_info = V12.inspect_ply(self.fine)
        archive_records_sha = V12.hashlib.sha256(
            self.addition_records.tobytes(order="C")
        ).hexdigest()
        geometry_contract = {
            "source_points": base_info.count,
            "base_points": base_info.count,
            "removed_base_count": 0,
            "candidate_points": geometry_info.count,
            "addition_count": len(self.addition_records),
            "base_sha256": V12.sha256_path(self.canonical_fine),
            "candidate_sha256": V12.sha256_path(self.geometry),
            "manifest_sha256": V12.sha256_path(self.geometry_manifest),
            "archive_sha256": V12.sha256_path(self.geometry_archive),
            "base_payload_sha256": V12._payload_sha256(base_info),
            "candidate_prefix_payload_sha256": V12._payload_sha256(
                geometry_info, count=base_info.count
            ),
            "archive_records_sha256": archive_records_sha,
            "candidate_suffix_sha256": V12._payload_sha256(
                geometry_info,
                start=base_info.count,
                count=len(self.addition_records),
            ),
        }
        fine_document = dict(geometry_document)
        fine_document.update({
            "schema_version": 1,
            "operation": "site1-v11-candidate-only-water-addition-scalar-enrichment",
            "status": "built",
            "candidate_only": True,
            "canonical_install_performed": False,
            "resolution_label": "2mm",
            "nominal_spacing_m": 0.002,
            "geometry_contract": geometry_contract,
            "geometry_manifest": {
                "path": str(self.geometry_manifest.resolve()),
                "sha256": V12.sha256_path(self.geometry_manifest),
                "archived_copy": geometry_copy.name,
                "archived_copy_sha256": V12.sha256_path(geometry_copy),
                "operation": None,
                "candidate": geometry_document["candidate"],
            },
            "input_fingerprints": {
                "base_water": upstream_fingerprint(self.canonical_fine),
                "geometry_candidate": upstream_fingerprint(self.geometry),
                "geometry_manifest": upstream_fingerprint(self.geometry_manifest),
                "geometry_archive": upstream_fingerprint(self.geometry_archive),
                "sand": upstream_fingerprint(self.sand_sentinel),
                "rock": upstream_fingerprint(self.rock_sentinel),
                "cleanmesh": upstream_fingerprint(self.cleanmesh),
                "normalization_manifest": upstream_fingerprint(self.normalization),
            },
            "scalar_enrichment_implementation": implementation_hashes(
                V12._FINE_IMPLEMENTATIONS
            ),
            "parameters": {
                "semantic": {
                    "resolution_label": "2mm",
                    "nominal_spacing_m": 0.002,
                    "coarse_geometry_source": "local-cleanmesh",
                    "minimum_combined_finite_fraction": 1.0,
                    "minimum_component_field_finite_fraction": 1.0,
                }
            },
            "scalar_enrichment": {
                "component_field_finite_coverage": (
                    V12.scalar_enrichment.verify_candidate_component_scalar_coverage(
                        self.fine,
                        base_points=base_info.count,
                        component_labels=self.component_labels,
                        context="release fixture",
                    )["coverage"]
                ),
                "minimum_component_field_finite_fraction": 1.0,
            },
            "candidate": {
                "path": str(self.fine.resolve()),
                "points": fine_info.count,
                "sha256": V12.sha256_path(self.fine),
                "base_payload_sha256": V12._payload_sha256(
                    fine_info, count=base_info.count
                ),
                "suffix_sha256": V12._payload_sha256(
                    fine_info,
                    start=base_info.count,
                    count=fine_info.count - base_info.count,
                ),
            },
            "invariants": {
                "geometry_candidate_verified_as_base_plus_archive": True,
                "existing_base_payload_byte_exact": True,
                "coordinates_and_normals_archive_exact": True,
                "colour_intensity_composition_archive_exact": True,
                "geometry_metrics_from_local_cleanmesh": True,
                "combined_metrics_use_v10_global_normalization": True,
                "geometry_component_membership_verified": True,
                "component_field_scalar_coverage_complete": True,
                "component_field_scalar_ranges_verified": True,
                "coarse_geometry_metrics_from_exact_fine_selection": False,
                "canonical_writes": False,
            },
        })
        self.fine_manifest = write_json(
            self.run / "water-final-2mm" / "manifest.json", fine_document
        )
        interface_document = {
            "schema_version": 1,
            "operation": "site1-v12-post-build-terrain-water-interface-audit",
            "status": "passed",
            "candidate_only": True,
            "canonical_writes": False,
            "annotations_are_search_neighbourhoods_not_masks": True,
            "terrain_resolution": {
                "selected": "canonical-1mm-SAND-plus-ROCK",
                "spacing_m": 0.001,
                "coarse_5mm_used": False,
            },
            "inputs": {
                "base_water": audit_fingerprint(self.canonical_fine, ply=True),
                "final_water": audit_fingerprint(self.fine, ply=True),
                "fine_manifest": audit_fingerprint(self.fine_manifest),
                "geometry_manifest": audit_fingerprint(self.geometry_manifest),
                "geometry_archive": audit_fingerprint(self.geometry_archive),
                "sand_1mm": audit_fingerprint(self.sand_sentinel, ply=True),
                "rock_1mm": audit_fingerprint(self.rock_sentinel, ply=True),
                "review_config": audit_fingerprint(self.review_config),
            },
            "implementations": {
                name: audit_fingerprint(V12.SCRIPT_DIR / name)
                for name in V12._INTERFACE_AUDIT_IMPLEMENTATIONS
            },
            "append_contract": {
                "source_base_points": base_info.count,
                "removed_base_points": 0,
                "base_points": base_info.count,
                "addition_count": len(self.addition_records),
                "final_points": fine_info.count,
                "base_payload_sha256": V12._payload_sha256(base_info),
                "final_prefix_payload_sha256": V12._payload_sha256(
                    fine_info, count=base_info.count
                ),
                "base_payload_byte_exact": True,
                "surviving_base_payload_byte_exact": True,
                "surviving_base_row_order_preserved": True,
                "suffix_xyz_archive_exact": True,
                "component_labels_present": [1],
            },
            "metrics": {
                "moving_circle_aggregate": {
                    "water_only_center_count_is_acceptance_criterion": False,
                },
                "density_continuity_lower_and_upper_gate": {
                    "circle_radius_m": 0.08,
                    "step_m": 0.08,
                    "minimum_ratio": 0.85,
                    "maximum_ratio": 1.25,
                    "post_build_lower_and_upper_bounds_passed": True,
                    "water_only_center_count_is_acceptance_criterion": True,
                },
            },
            "acceptance": {
                "checks": {
                    name: True for name in V12._INTERFACE_AUDIT_CHECKS
                },
                "passed": True,
                "water_only_center_count_is_acceptance_criterion": True,
            },
        }
        interface_document["manifest_lock"] = {
            "method": "sha256-canonical-json-excluding-manifest_lock",
            "sha256": V12._canonical_json_hash_without_key(
                interface_document, "manifest_lock"
            ),
        }
        self.interface_audit_manifest = write_json(
            self.run / "interface-audit" / "manifest.json",
            interface_document,
        )
        self.report = write_report(
            self.run / "water-final-5mm" / "downsample-report.json",
            self.fine,
            self.coarse,
        )
        native = V12.verify_cleanmesh_downsample_report(
            self.report, self.fine, self.coarse
        )
        subset = V12.verify_ordered_record_subsequence(self.fine, self.coarse)
        self.downsample_manifest = write_json(
            self.run / "water-final-5mm" / "stage-manifest.json",
            {
                "schema_version": 1,
                "operation": "site1-v12-native-fine-to-coarse-downsample",
                "candidate_only": True,
                "fine_first": True,
                "coarse_scalar_recalculation_performed": False,
                "minimum_spacing_m": 0.005,
                "command": [
                    str(self.downsample_executable.resolve()),
                    "--input", str(self.fine.resolve()),
                    "--output", str(self.coarse.resolve()),
                    "--spacing", "0.005",
                    "--report", str(self.report.resolve()),
                    "--chunk-points", "3",
                ],
                "executable": V12.file_fingerprint(
                    self.downsample_executable, ply=False
                ),
                "fine_candidate": V12.file_fingerprint(self.fine),
                "coarse_candidate": V12.file_fingerprint(self.coarse),
                "native_report": V12.file_fingerprint(self.report, ply=False),
                "verification": {
                    "native_report": native,
                    "exact_ordered_subsequence": subset,
                },
            },
        )
        self.args = SimpleNamespace(
            data_dir=self.data,
            run_dir=self.run,
            release_dir=self.run / "release",
            candidate_2mm=self.fine,
            candidate_5mm=self.coarse,
            downsample_report=self.report,
            fine_manifest=self.fine_manifest,
            geometry_manifest=self.geometry_manifest,
            geometry_archive=self.geometry_archive,
            interface_audit_manifest=self.interface_audit_manifest,
            downsample_manifest=self.downsample_manifest,
            review_config=self.review_config,
            normalization_manifest=self.normalization,
            cleanmesh=self.cleanmesh,
            downsample=self.downsample_executable,
        )


def compact_fixture_interface_audit_verifier(**kwargs):
    """Bound verifier seam for release fixtures without a full audit raster."""

    manifest = Path(kwargs["manifest_path"]).resolve()
    return {
        "verified": True,
        "status": "passed",
        "manifest": str(manifest),
        "manifest_sha256": V12.sha256_path(manifest),
    }


def build_fixture(fixture: Fixture):
    return V12.build(
        fixture.args,
        interface_audit_verifier=compact_fixture_interface_audit_verifier,
    )


class PlyAndSubsequenceTests(unittest.TestCase):
    def test_header_differences_are_allowed_for_exact_ordered_payload_subset(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            result = V12.verify_ordered_record_subsequence(
                fixture.fine, fixture.coarse, chunk_records=3
            )
            self.assertTrue(result["verified"])
            self.assertEqual(result["matched_points"], len(fixture.coarse_records))
            self.assertEqual(result["last_matched_fine_index"], 12)
            self.assertNotEqual(
                V12.inspect_ply(fixture.fine).offset,
                V12.inspect_ply(fixture.coarse).offset,
            )

    def test_changed_non_xyz_bytes_fail_full_record_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            changed = fixture.coarse_records.copy()
            changed[2]["scalar_Intensity"] += 1.0
            bad = write_ply(
                fixture.run / "bad-full-record.ply", changed, comment="bad scalar"
            )
            with self.assertRaisesRegex(RuntimeError, "not an ordered full-record"):
                V12.verify_ordered_record_subsequence(
                    fixture.fine, bad, chunk_records=4
                )

    def test_reordered_records_fail_ordered_proof(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            reordered = fixture.coarse_records[[0, 3, 2, 4]].copy()
            bad = write_ply(
                fixture.run / "bad-order.ply", reordered, comment="bad order"
            )
            with self.assertRaisesRegex(RuntimeError, "not an ordered full-record"):
                V12.verify_ordered_record_subsequence(
                    fixture.fine, bad, chunk_records=2
                )

    def test_trailing_payload_bytes_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            with fixture.fine.open("ab") as handle:
                handle.write(b"x")
            with self.assertRaisesRegex(ValueError, "fixed payload size"):
                V12.inspect_ply(fixture.fine)

    def test_coarse_scalar_audit_rejects_an_omitted_fine_component(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base = records(2, start=10.0, scan_id=2.0)
            additions = records(4, start=20.0)
            fine_records = np.concatenate((base, additions))
            fine = write_ply(root / "fine.ply", fine_records)
            # The active 5 mm subset contains additions from label 1 only;
            # label 2 must not silently disappear from the release audit.
            coarse = write_ply(
                root / "coarse.ply",
                fine_records[[0, 2, 3]].copy(),
            )
            with self.assertRaisesRegex(
                RuntimeError,
                "omits fine scalar component labels: \\[2\\]",
            ):
                V12.scalar_enrichment.verify_coarse_exact_subset_component_scalar_coverage(
                    fine,
                    coarse,
                    fine_base_points=len(base),
                    fine_component_labels=np.asarray([1, 1, 2, 2], np.int32),
                    chunk_records=2,
                )


class CleanMeshReportTests(unittest.TestCase):
    def test_native_report_counts_spacing_and_stride_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            result = V12.verify_cleanmesh_downsample_report(
                fixture.report, fixture.fine, fixture.coarse
            )
            self.assertTrue(result["verified"])
            self.assertEqual(result["source_points"], 13)
            self.assertEqual(result["output_points"], 5)
            self.assertEqual(result["record_stride"], DTYPE.itemsize)

    def test_native_report_mismatches_fail_closed(self):
        changes = (
            {"source_points": 12},
            {"output_points": 4},
            {"minimum_spacing_m": 0.004999},
            {"record_stride": DTYPE.itemsize + 1},
            {"non_finite_positions": 1},
        )
        for index, change in enumerate(changes):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                fixture = Fixture(directory)
                report = write_report(
                    fixture.run / f"bad-report-{index}.json",
                    fixture.fine,
                    fixture.coarse,
                    **change,
                )
                with self.assertRaises(RuntimeError):
                    V12.verify_cleanmesh_downsample_report(
                        report, fixture.fine, fixture.coarse
                    )

    def test_report_must_name_the_exact_candidate_files(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            other = write_ply(
                fixture.run / "other.ply", fixture.fine_records, comment="other"
            )
            report = write_report(
                fixture.run / "wrong-path-report.json",
                fixture.fine,
                fixture.coarse,
                input=str(other),
            )
            with self.assertRaisesRegex(RuntimeError, "not the fine candidate"):
                V12.verify_cleanmesh_downsample_report(
                    report, fixture.fine, fixture.coarse
                )


class FingerprintCompletenessTests(unittest.TestCase):
    def test_release_interface_audit_schema_matches_hardened_audit(self):
        self.assertEqual(
            V12._INTERFACE_AUDIT_IMPLEMENTATIONS,
            (
                "site1_v12_interface_audit.py",
                "site1_v11_water_density.py",
                "site1_v11_water_scalar_enrichment.py",
                "site1_v12_water_pipeline.py",
                "rebuild_site1_fossils_v10.py",
                "rebuild_site1_fossils_v9.py",
                "rebuild_site1_fossils_water.py",
                "site1_v11_confidence.py",
                "site1_v12_water_refinement.py",
                "site1_v11_terrain.py",
            ),
        )
        self.assertEqual(
            V12._INTERFACE_AUDIT_CHECKS,
            {
                "append_contract",
                "final_additions_are_exact_vacant_safe_reservoir_rows",
                "final_addition_stored_coordinate_geometry_passed",
                "terrain_edge_eligibility_is_candidate_independent",
                "terrain_edge_meaningful_configured_support_continuity_passed",
                "measured_density_lower_and_upper_bounds_passed",
            },
        )
        self.assertTrue(all(
            (V12.SCRIPT_DIR / name).is_file()
            for name in V12._INTERFACE_AUDIT_IMPLEMENTATIONS
        ))

    def test_empty_and_partial_fingerprints_never_compare_equal(self):
        self.assertFalse(V12._same_fingerprint({}, {}))
        self.assertFalse(
            V12._same_fingerprint(
                {"bytes": 1, "sha256": "a" * 64},
                {"bytes": 1},
            )
        )

    def test_assert_requires_generic_and_complete_ply_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            report = V12.file_fingerprint(fixture.report, ply=False)
            report.pop("sha256")
            with self.assertRaisesRegex(RuntimeError, "missing required keys.*sha256"):
                V12._assert_fingerprint(fixture.report, report, "partial report")

            fine = V12.file_fingerprint(fixture.fine)
            fine.pop("schema")
            with self.assertRaisesRegex(RuntimeError, "missing required keys.*schema"):
                V12._assert_fingerprint(fixture.fine, fine, "partial fine PLY")


class FarLobeProvenanceTests(unittest.TestCase):
    def test_performed_far_lobe_decision_binds_exact_reversible_archives(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_records = records(6, start=100.0, scan_id=2.0)
            source = write_ply(root / "source.ply", source_records)
            source_info = V12.inspect_ply(source)
            removed_indices = np.asarray([1, 4], dtype="<i8")
            record_archive = root / "removed-records.bin"
            index_archive = root / "removed-indices.i64"
            record_archive.write_bytes(
                source_records[removed_indices].tobytes(order="C")
            )
            index_archive.write_bytes(removed_indices.tobytes(order="C"))
            index_fingerprint = upstream_fingerprint(index_archive)
            index_fingerprint["points"] = len(removed_indices)
            decision = {
                "performed": True,
                "reversible": True,
                "measured_no_eligible_component": False,
                "reason": "detached component passed measured gates",
                "seed_xy": [1.0, 2.0],
                "maximum_seed_distance_m": 2.0,
                "grid_pitch_m": 0.025,
                "bridge_radius_m": 0.05,
                "bridge_iterations": 3,
                "detachment_gap_m": 0.15,
                "maximum_component_fraction": 0.25,
                "occupied_cell_count": 10,
                "selected_occupied_cell_count": 2,
                "largest_occupied_cell_count": 8,
                "seed_distance_m": 0.1,
                "component_fraction": 0.2,
                "minimum_cell_center_separation_m": 0.2,
                "minimum_point_separation_lower_bound_m": 0.17,
                "removed_count": 2,
                "source_count_before": 6,
                "surviving_source_count": 4,
                "surviving_source_payload_byte_exact": True,
                "surviving_source_row_order_preserved": True,
                "archive": {
                    "removed_count": 2,
                    "record_stride_bytes": DTYPE.itemsize,
                    "record_archive_format": "raw-fixed-stride-source-dtype",
                    "record_dtype_descr": [list(item) for item in DTYPE.descr],
                    "records": upstream_fingerprint(record_archive),
                    "source_indices": index_fingerprint,
                    "source_index_dtype": "<i8",
                    "exact_source_rows_archived": True,
                    "exact_source_indices_archived": True,
                },
            }
            result = V12._verify_far_lobe_provenance(
                decision,
                base_info=source_info,
            )
            self.assertTrue(result["performed"])
            self.assertEqual(result["removed_count"], 2)
            self.assertEqual(result["surviving_source_count"], 4)

            decision["component_fraction"] = 0.21
            with self.assertRaisesRegex(RuntimeError, "fraction disagrees"):
                V12._verify_far_lobe_provenance(
                    decision,
                    base_info=source_info,
                )


class UpstreamProvenanceTests(unittest.TestCase):
    def test_build_hash_locks_all_compact_upstream_stages(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            manifest = json.loads(
                (fixture.run / "release" / "manifest.json").read_text()
            )
            provenance = manifest["upstream_provenance"]
            self.assertTrue(provenance["verified"])
            self.assertTrue(provenance["fine_first"])
            self.assertFalse(provenance["coarse_scalar_recalculation"])
            self.assertEqual(provenance["interface_audit"]["status"], "passed")
            self.assertEqual(
                provenance["interface_audit"]["independent_gate_recomputation"],
                {
                    "verified": True,
                    "status": "passed",
                    "manifest_sha256": V12.sha256_path(
                        fixture.interface_audit_manifest
                    ),
                },
            )
            self.assertEqual(
                set(provenance["artifacts"]),
                set(V12._PROVENANCE_SNAPSHOT_NAMES),
            )
            for row in provenance["artifacts"].values():
                self.assertTrue(Path(row["snapshot"]["path"]).is_file())
                self.assertTrue(
                    V12._same_fingerprint(row["source"], row["snapshot"])
                )

    def test_missing_stale_or_failed_interface_audit_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.interface_audit_manifest.unlink()
            with self.assertRaises(FileNotFoundError):
                build_fixture(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.interface_audit_manifest.read_text())
            document["inputs"]["final_water"]["sha256"] = "0" * 64
            document["manifest_lock"]["sha256"] = V12._canonical_json_hash_without_key(
                document, "manifest_lock"
            )
            write_json(fixture.interface_audit_manifest, document)
            with self.assertRaisesRegex(RuntimeError, "final_water hash drift"):
                build_fixture(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.interface_audit_manifest.read_text())
            document["acceptance"]["checks"][
                "measured_density_lower_and_upper_bounds_passed"
            ] = False
            document["acceptance"]["passed"] = False
            document["manifest_lock"]["sha256"] = V12._canonical_json_hash_without_key(
                document, "manifest_lock"
            )
            write_json(fixture.interface_audit_manifest, document)
            with self.assertRaisesRegex(RuntimeError, "acceptance checks"):
                build_fixture(fixture)

    def test_interface_audit_partial_fingerprint_is_rejected_even_when_relocked(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.interface_audit_manifest.read_text())
            document["inputs"]["fine_manifest"].pop("sha256")
            document["manifest_lock"]["sha256"] = V12._canonical_json_hash_without_key(
                document, "manifest_lock"
            )
            write_json(fixture.interface_audit_manifest, document)
            with self.assertRaisesRegex(RuntimeError, "fingerprint key set differs"):
                build_fixture(fixture)

    def test_direct_build_recomputes_and_rejects_relocked_forged_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.interface_audit_manifest.read_text())
            # This extra passed evidence is deliberately outside the compact
            # release subset.  Its self-lock can be recomputed coherently, but
            # the hardened audit verifier must rebuild the evidence rather
            # than accepting that lock as authenticity.
            document["audit_parameters"] = {"edge_sample_limit": 17}
            document["metrics"]["forged_relocked_gate"] = {
                "passed": True,
            }
            document["manifest_lock"][
                "sha256"
            ] = V12._canonical_json_hash_without_key(document, "manifest_lock")
            write_json(fixture.interface_audit_manifest, document)
            self.assertEqual(
                document["manifest_lock"]["sha256"],
                V12._canonical_json_hash_without_key(document, "manifest_lock"),
            )

            recomputed = json.loads(json.dumps(document))
            recomputed["metrics"].pop("forged_relocked_gate")
            recomputed["manifest_lock"][
                "sha256"
            ] = V12._canonical_json_hash_without_key(
                recomputed,
                "manifest_lock",
            )

            def rebuild_without_forged_evidence(**kwargs):
                write_json(Path(kwargs["output_path"]), recomputed)
                return {"verified": True}

            with mock.patch.object(
                V12.interface_audit,
                "verify_interface_audit",
                wraps=V12.interface_audit.verify_interface_audit,
            ) as verifier, mock.patch.object(
                V12.interface_audit,
                "build_interface_audit",
                side_effect=rebuild_without_forged_evidence,
            ) as rebuild:
                with self.assertRaisesRegex(
                    RuntimeError,
                    "differs from independent gate recomputation",
                ):
                    # No fixture override: this is the direct/default release
                    # path used by the CLI.
                    V12.build(fixture.args)
            verifier.assert_called_once()
            rebuild.assert_called_once()

    def test_arbitrary_same_schema_candidates_are_rejected_even_with_new_report(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.fine.unlink()
            arbitrary = records(13, start=910.0)
            write_ply(fixture.fine, arbitrary, comment="arbitrary same schema fine")
            fixture.coarse.unlink()
            write_ply(
                fixture.coarse,
                arbitrary[[0, 2, 5, 8, 12]].copy(),
                comment="consistent arbitrary coarse",
            )
            write_report(fixture.report, fixture.fine, fixture.coarse)
            with self.assertRaisesRegex(
                RuntimeError,
                "fine scalar candidate hash drift|changed existing WATER payload",
            ):
                build_fixture(fixture)

    def test_geometry_source_and_downsample_policy_tampering_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.geometry_manifest.read_text())
            document["source"].pop("sha256")
            write_json(fixture.geometry_manifest, document)
            with self.assertRaisesRegex(
                RuntimeError,
                "geometry manifest source hash does not match supplied file|"
                "geometry source WATER fingerprint is incomplete",
            ):
                build_fixture(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.downsample_manifest.read_text())
            document["coarse_scalar_recalculation_performed"] = True
            write_json(fixture.downsample_manifest, document)
            with self.assertRaisesRegex(RuntimeError, "coarse scalar recalculation"):
                build_fixture(fixture)

    def test_fine_component_scalar_attestation_is_mandatory(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.fine_manifest.read_text())
            document["invariants"].pop(
                "component_field_scalar_coverage_complete"
            )
            write_json(fixture.fine_manifest, document)
            with self.assertRaisesRegex(RuntimeError, "fine scalar invariant"):
                build_fixture(fixture)

    def test_far_lobe_no_component_attestation_must_match_measurements(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            document = json.loads(fixture.geometry_manifest.read_text())
            document["far_lobe_cull"]["selected_occupied_cell_count"] = 1
            write_json(fixture.geometry_manifest, document)
            with self.assertRaisesRegex(
                RuntimeError,
                "measured no-component proof",
            ):
                build_fixture(fixture)


class ReleaseLifecycleTests(unittest.TestCase):
    def test_build_snapshots_installs_only_water_and_restores_from_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            old01_before = V12.file_fingerprint(fixture.old01)
            sand_before = fixture.sand_sentinel.read_bytes()
            source_fine_before = V12.file_fingerprint(fixture.canonical_fine)
            source_coarse_before = V12.file_fingerprint(fixture.canonical_coarse)
            candidate_fine_before = V12.file_fingerprint(fixture.fine)
            candidate_coarse_before = V12.file_fingerprint(fixture.coarse)

            built = build_fixture(fixture)
            self.assertTrue(built["built"])
            manifest = json.loads(
                (fixture.run / "release" / "manifest.json").read_text()
            )
            self.assertEqual(set(manifest["clouds"]), {"WATER-2mm", "WATER-5mm"})
            for label, expected in (
                ("WATER-2mm", source_fine_before),
                ("WATER-5mm", source_coarse_before),
            ):
                snapshot = Path(manifest["clouds"][label]["snapshot"]["path"])
                self.assertTrue(V12._same_fingerprint(V12.file_fingerprint(snapshot), expected))

            with mock.patch.object(V12, "refuse_running_app"):
                installed = V12.install(fixture.args)
            self.assertTrue(installed["installed"])
            self.assertFalse(fixture.fine.exists())
            self.assertFalse(fixture.coarse.exists())
            self.assertTrue(
                V12._same_fingerprint(
                    V12.file_fingerprint(fixture.canonical_fine),
                    candidate_fine_before,
                )
            )
            self.assertTrue(
                V12._same_fingerprint(
                    V12.file_fingerprint(fixture.canonical_coarse),
                    candidate_coarse_before,
                )
            )

            # Post-install retirement may remove the multi-gigabyte geometry
            # candidate.  Verification must use the protected compact
            # provenance snapshots and active canonical candidate hashes.
            fixture.geometry.unlink()
            installed_verification = V12.verify(fixture.args)
            self.assertTrue(installed_verification["verified"])
            self.assertTrue(
                installed_verification["interface_audit_provenance_verified"]
            )

            # Damage the install transaction's mutable previous backups.  A
            # restore must still succeed because v12 materialises it from the
            # hash-locked release snapshots.
            install_previous = Path(installed["transaction"]["archives"]["WATER-2mm"])
            install_previous.write_bytes(b"not a PLY any more")
            with mock.patch.object(V12, "refuse_running_app"):
                restored = V12.restore(fixture.args)
            self.assertTrue(restored["restored"])
            self.assertEqual(
                restored["transaction"]["restore_source"],
                "release source snapshots",
            )
            self.assertTrue(
                V12._same_fingerprint(
                    V12.file_fingerprint(fixture.canonical_fine),
                    source_fine_before,
                )
            )
            self.assertTrue(
                V12._same_fingerprint(
                    V12.file_fingerprint(fixture.canonical_coarse),
                    source_coarse_before,
                )
            )
            self.assertTrue(
                V12._same_fingerprint(V12.file_fingerprint(fixture.fine), candidate_fine_before)
            )
            self.assertTrue(
                V12._same_fingerprint(V12.file_fingerprint(fixture.coarse), candidate_coarse_before)
            )
            self.assertEqual(fixture.sand_sentinel.read_bytes(), sand_before)
            self.assertEqual(V12.file_fingerprint(fixture.old01), old01_before)
            self.assertTrue(V12.verify(fixture.args)["verified"])

    def test_build_refuses_overwrite_and_candidate_hardlink_alias(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            with self.assertRaisesRegex(FileExistsError, "release directory"):
                build_fixture(fixture)

        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            fixture.fine.unlink()
            os.link(fixture.canonical_fine, fixture.fine)
            write_report(fixture.report, fixture.fine, fixture.coarse)
            with self.assertRaisesRegex(RuntimeError, "hard-link aliases"):
                build_fixture(fixture)

    def test_restore_refuses_to_overwrite_reoccupied_candidate_path(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            with mock.patch.object(V12, "refuse_running_app"):
                V12.install(fixture.args)
            canonical_before = V12.file_fingerprint(fixture.canonical_fine)
            fixture.fine.write_bytes(b"unrelated occupant")
            with (
                mock.patch.object(V12, "refuse_running_app"),
                self.assertRaisesRegex(RuntimeError, "unexpectedly occupied"),
            ):
                V12.restore(fixture.args)
            self.assertEqual(V12.file_fingerprint(fixture.canonical_fine), canonical_before)

    def test_failed_restore_preparation_leaves_no_unjournaled_clone_orphan(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            with mock.patch.object(V12, "refuse_running_app"):
                V12.install(fixture.args)
            transactions = fixture.run / "release" / "transactions"
            before = set(transactions.iterdir())
            real_clone = V12._clone_or_copy
            calls = 0

            def fail_second_clone(source, destination):
                nonlocal calls
                calls += 1
                if calls == 2:
                    raise RuntimeError("fixture second clone failure")
                return real_clone(source, destination)

            with (
                mock.patch.object(V12, "refuse_running_app"),
                mock.patch.object(V12, "_clone_or_copy", side_effect=fail_second_clone),
                self.assertRaisesRegex(RuntimeError, "second clone failure"),
            ):
                V12.restore(fixture.args)
            self.assertEqual(set(transactions.iterdir()), before)

    def test_manifest_candidate_path_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            manifest_path = fixture.run / "release" / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["clouds"]["WATER-2mm"]["candidate_path"] = str(
                fixture.canonical_fine
            )
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
            with self.assertRaisesRegex(RuntimeError, "candidate path mismatch"):
                V12.verify(fixture.args)

    def test_release_manifest_missing_fingerprint_keys_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            manifest_path = fixture.run / "release" / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["clouds"]["WATER-2mm"]["candidate"] = {}
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
            with self.assertRaisesRegex(RuntimeError, "missing required keys"):
                V12.verify(fixture.args)

    def test_release_manifest_cannot_forge_component_scalar_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            manifest_path = fixture.run / "release" / "manifest.json"
            document = json.loads(manifest_path.read_text())
            coverage = document["upstream_provenance"][
                "fine_scalar_enrichment"
            ]["component_field_scalar_coverage"]["coverage"]
            component = coverage["components"][0]
            field = component["fields"][coverage["required_fields"][0]]
            field["finite"] -= 1
            field["fraction"] = field["finite"] / field["total"]
            write_json(manifest_path, document)
            with self.assertRaisesRegex(RuntimeError, "not 100% finite"):
                V12.verify(fixture.args)


class InterruptedTransactionRecoveryTests(unittest.TestCase):
    def test_startup_recovery_rolls_back_partial_install(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            paths = V12._coerce_paths(fixture.args)
            manifest_path, manifest = V12._read_release(paths)
            transaction_dir = V12._new_transaction_dir(paths, "install")
            transaction_dir.mkdir()
            (transaction_dir / "previous").mkdir()
            items = V12._journal_items(
                paths, manifest, transaction_dir, "install"
            )
            journal_path = transaction_dir / "journal.json"
            journal = {
                "schema_version": V12.JOURNAL_SCHEMA_VERSION,
                "operation": V12.JOURNAL_OPERATION,
                "action": "install",
                "source_status": "built",
                "target_status": "installed",
                "state": "archived-current",
                "created": V12._now(),
                "items": [
                    {
                        "label": item.label,
                        "canonical": str(item.canonical),
                        "replacement": str(item.replacement),
                        "archive": str(item.archive),
                        "expected_current": dict(item.expected_current),
                        "expected_replacement": dict(item.expected_replacement),
                        "phase": "intent",
                    }
                    for item in items
                ],
                "events": [],
            }
            # Simulate a process crash after archiving only the fine canonical.
            fine_item = next(item for item in items if item.label == "WATER-2mm")
            V12._durable_replace(fine_item.canonical, fine_item.archive)
            V12._write_journal(journal_path, journal)
            self.assertFalse(fine_item.canonical.exists())

            with mock.patch.object(V12, "refuse_running_app"):
                recovered = V12.recover_incomplete_transactions(paths)
            self.assertEqual(recovered[0]["outcome"], "rolled-back-after-recovery")
            self.assertTrue(fine_item.canonical.exists())
            self.assertTrue(fixture.fine.exists())
            self.assertEqual(
                json.loads(journal_path.read_text())["state"],
                "rolled-back-after-recovery",
            )
            self.assertEqual(
                json.loads(manifest_path.read_text())["status"], "built"
            )

    def test_ambiguous_recovery_content_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory)
            build_fixture(fixture)
            paths = V12._coerce_paths(fixture.args)
            _, manifest = V12._read_release(paths)
            transaction_dir = V12._new_transaction_dir(paths, "install")
            transaction_dir.mkdir()
            (transaction_dir / "previous").mkdir()
            items = V12._journal_items(paths, manifest, transaction_dir, "install")
            journal_path = transaction_dir / "journal.json"
            journal = {
                "schema_version": V12.JOURNAL_SCHEMA_VERSION,
                "operation": V12.JOURNAL_OPERATION,
                "action": "install",
                "source_status": "built",
                "target_status": "installed",
                "state": "archived-current",
                "created": V12._now(),
                "items": [
                    {
                        "label": item.label,
                        "canonical": str(item.canonical),
                        "replacement": str(item.replacement),
                        "archive": str(item.archive),
                        "expected_current": dict(item.expected_current),
                        "expected_replacement": dict(item.expected_replacement),
                        "phase": "intent",
                    }
                    for item in items
                ],
                "events": [],
            }
            fine_item = next(item for item in items if item.label == "WATER-2mm")
            V12._durable_replace(fine_item.canonical, fine_item.archive)
            fine_item.canonical.write_bytes(b"unknown external content")
            unknown = fine_item.canonical.read_bytes()
            V12._write_journal(journal_path, journal)
            with (
                mock.patch.object(V12, "refuse_running_app"),
                self.assertRaisesRegex(RuntimeError, "unknown content"),
            ):
                V12.recover_incomplete_transactions(paths)
            self.assertEqual(fine_item.canonical.read_bytes(), unknown)


if __name__ == "__main__":
    unittest.main()
