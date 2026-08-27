import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import site1_v11_hole_pipeline as pipeline


def dtype():
    return np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1"),
        ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4"),
        ("scalar_Intensity", "<f4"), ("scalar_Composite", "<f4"),
        ("scalar_ScanID", "<f4"),
    ])


def write_ply(path, records):
    names = {
        "f": "float", "u": "uchar", "i": "int",
    }
    with open(path, "wb") as handle:
        handle.write(b"ply\nformat binary_little_endian 1.0\n")
        handle.write(f"element vertex {len(records)}\n".encode())
        for name in records.dtype.names:
            field = records.dtype[name]
            kind = names[field.kind]
            handle.write(f"property {kind} {name}\n".encode())
        handle.write(b"end_header\n")
        records.tofile(handle)


class HolePipelineTests(unittest.TestCase):
    def ring(self):
        angle = np.linspace(0, 2 * np.pi, 300, endpoint=False)
        records = np.zeros(len(angle), dtype())
        records["x"] = 0.5 + 0.30 * np.cos(angle)
        records["y"] = 0.5 + 0.30 * np.sin(angle)
        records["z"] = 2.0 + 0.01 * records["x"]
        records["nx"], records["ny"], records["nz"] = 0, 0, 1
        records["scalar_ScanID"] = 999
        records["scalar_Intensity"] = 5000
        records["scalar_Composite"] = 100
        return records

    def test_surface_agreement_accepts_matching_models(self):
        donor = self.ring()
        q = np.array([[0.45, 0.45], [0.50, 0.50], [0.55, 0.55]])
        result = pipeline.validate_surface_agreement(
            q,
            np.column_stack((donor["x"], donor["y"], donor["z"])),
            lambda x, y: 2.0 + 0.01 * x,
        )
        self.assertTrue(result.accepted)
        self.assertLess(result.maximum_spread_m, 1e-4)

    def test_component_surface_checks_do_not_mix_pool_levels(self):
        policy = pipeline.holes.SeededHolePolicy(
            diagnostic_cell_m=0.05,
            water_support_radius_m=0.04,
            terrain_support_radius_m=0.01,
            seed_search_radius_m=0.20,
            minimum_area_m2=0.001,
            maximum_area_m2=0.20,
            sector_radius_m=0.25,
            minimum_support_sectors=4,
            minimum_water_sectors=4,
        )
        angle = np.linspace(0, 2 * np.pi, 160, endpoint=False)
        low = np.column_stack((
            0.35 + 0.13 * np.cos(angle),
            0.50 + 0.13 * np.sin(angle),
        ))
        high = np.column_stack((
            0.75 + 0.13 * np.cos(angle),
            0.50 + 0.13 * np.sin(angle),
        ))
        plan = pipeline.holes.detect_seeded_holes(
            np.vstack((low, high)), np.empty((0, 2)),
            review_bbox=(0.1, 1.0, 0.2, 0.8),
            seeds={"low": (0.35, 0.50), "high": (0.75, 0.50)},
            policy=policy,
        )
        self.assertEqual(len(plan.accepted_labels), 2)
        # Use actual grid-cell centres so label lookup is exact.
        candidate = []
        for item in plan.holes:
            if not item.accepted:
                continue
            row, col = np.unravel_index(
                item.cell_indices[len(item.cell_indices) // 2],
                plan.grid.labels.shape,
            )
            candidate.append((plan.grid.x_centres[col], plan.grid.y_centres[row]))
        candidate = np.asarray(candidate)
        donor = np.vstack((
            np.column_stack((low, np.full(len(low), 2.0))),
            np.column_stack((high, np.full(len(high), 2.2))),
        ))
        result, rows = pipeline.validate_component_surfaces(
            plan, candidate, donor,
            lambda x, y: np.where(x < 0.55, 2.0, 2.2),
            donor_margin_m=0.05,
        )
        self.assertTrue(result.accepted)
        self.assertEqual(len(rows), 2)
        drifted = donor.copy()
        drifted[:, 2] += 0.02
        rejected, _ = pipeline.validate_component_surfaces(
            plan, candidate, drifted,
            lambda x, y: np.where(x < 0.55, 2.0, 2.2),
            donor_margin_m=0.05,
        )
        self.assertFalse(rejected.accepted)

    def test_new_records_preserve_donor_scalars_and_set_geometry(self):
        donor = self.ring()
        q = np.array([[0.48, 0.49], [0.53, 0.51]])
        def surface(x, y):
            return 2.0 + 0.01 * x, np.tile([0.0, 0.0, 1.0], (len(x), 1))
        output, index = pipeline.records_from_nearest_water(q, donor, surface)
        np.testing.assert_allclose(output["x"], q[:, 0])
        np.testing.assert_allclose(output["z"], 2.0 + 0.01 * q[:, 0])
        self.assertTrue(np.all(output["scalar_Intensity"] == 5000))
        self.assertTrue(np.all(output["scalar_ScanID"] == 999))
        self.assertEqual(len(index), len(q))

    def test_append_keeps_existing_payload_exact(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.ply"
            output = root / "candidate.ply"
            records = self.ring()[:10]
            additions = self.ring()[10:12]
            write_ply(source, records)
            report = pipeline.append_candidate_records(source, additions, output)
            source_info = pipeline.density.inspect_fixed_stride_ply(source)
            output_info = pipeline.density.inspect_fixed_stride_ply(output)
            source_mm = np.memmap(source, dtype=source_info.dtype, mode="r", offset=source_info.offset, shape=(10,))
            output_mm = np.memmap(output, dtype=output_info.dtype, mode="r", offset=output_info.offset, shape=(12,))
            np.testing.assert_array_equal(source_mm, output_mm[:10])
            self.assertEqual(report["candidate_points"], 12)

    def test_ply_fingerprint_detects_source_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source.ply"
            records = self.ring()[:10]
            write_ply(path, records)
            fingerprint = pipeline.fingerprint_ply(path)
            pipeline.assert_fingerprint_unchanged(fingerprint)
            records[0]["z"] += 0.25
            write_ply(path, records)
            with self.assertRaisesRegex(RuntimeError, "source changed"):
                pipeline.assert_fingerprint_unchanged(fingerprint)

    def test_load_review_uses_world_seeds(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({
                "plan_annotations": {"cyan_southern_gap": {"review_bbox": [0, 1, 0, 1]}},
                "marked_locations": {"image_2": [
                    {"id": key, "world": [0.4 + i * 0.01, 0.5]}
                    for i, key in enumerate(pipeline.DEFAULT_HOLE_MARK_IDS)
                ]},
            }))
            bbox, seeds = pipeline.load_hole_review(path)
            self.assertEqual(bbox, (0.0, 1.0, 0.0, 1.0))
            self.assertEqual(tuple(seeds), pipeline.DEFAULT_HOLE_MARK_IDS)

    def test_reference_provenance_is_hash_locked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entries = {}
            for key in ("surface_archive", "surface_config", "implementation"):
                path = root / key
                path.write_bytes(key.encode("ascii"))
                entries[key] = {
                    "path": str(path),
                    "sha256": pipeline.sha256_path(path),
                }
            provenance = {
                **entries,
                "callable": "rebuild_site1_fossils_v10.surface_values",
                "noise_scale": 1.0,
            }
            pipeline._verify_reference_provenance(provenance)
            Path(entries["surface_archive"]["path"]).write_bytes(b"changed")
            with self.assertRaisesRegex(RuntimeError, "provenance drift"):
                pipeline._verify_reference_provenance(provenance)

    def test_artifact_paths_reject_resolved_aliases_and_protected_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.ply"
            source.write_bytes(b"protected-source")
            output = root / "candidate.ply"
            alias = root / "candidate-link.json"
            alias.symlink_to(output.name)
            with self.assertRaisesRegex(ValueError, "distinct"):
                pipeline._validated_artifact_paths(
                    output_path=output,
                    audit_path=alias,
                    archive_path=root / "additions.npz",
                    protected_paths=[source],
                )
            with self.assertRaisesRegex(ValueError, "protected"):
                pipeline._validated_artifact_paths(
                    output_path=output,
                    audit_path=source,
                    archive_path=root / "additions.npz",
                    protected_paths=[source],
                )
            self.assertEqual(source.read_bytes(), b"protected-source")

    def test_artifact_paths_reject_every_site1_canonical_role(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for role in ("SAND", "ROCK", "VEG", "WATER"):
                with self.subTest(role=role), self.assertRaisesRegex(
                    ValueError, "canonical"
                ):
                    pipeline._validated_artifact_paths(
                        output_path=root / "candidate.ply",
                        audit_path=root / f"Site1-{role}-5mm.ply",
                        archive_path=root / "additions.npz",
                        protected_paths=[],
                    )

    def test_fine_and_coarse_runners_reject_artifact_aliases_at_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shared = root / "shared-output"
            common = {
                "source_water_path": root / "missing-water.ply",
                "terrain_paths": [],
                "config_path": root / "missing-config.json",
                "output_path": shared,
                "audit_path": shared,
                "archive_path": root / "additions.npz",
                "spacing_m": 0.005,
                "reference_surface": lambda x, y: (
                    np.zeros(len(x)),
                    np.tile([0.0, 0.0, 1.0], (len(x), 1)),
                ),
                "reference_provenance": {},
                "overwrite": True,
            }
            with self.assertRaisesRegex(ValueError, "distinct"):
                pipeline.run_hole_completion(**common)
            with self.assertRaisesRegex(ValueError, "distinct"):
                pipeline.run_coarse_hole_completion_from_fine(
                    **common,
                    fine_manifest_path=root / "missing-fine-manifest.json",
                )

    def test_coarse_holes_copy_exact_fine_geometry_and_coarse_donor_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "review.json"
            config.write_text("{}\n")
            provenance = {}
            for key in ("surface_archive", "surface_config", "implementation"):
                path = root / key
                path.write_bytes(key.encode("ascii"))
                provenance[key] = {
                    "path": str(path),
                    "sha256": pipeline.sha256_path(path),
                }
            provenance.update({
                "callable": "rebuild_site1_fossils_v10.surface_values",
                "noise_scale": 1.0,
            })

            coarse_source = root / "coarse-base.ply"
            write_ply(coarse_source, self.ring())
            terrain = root / "terrain.ply"
            write_ply(terrain, np.empty(0, dtype()))

            gx, gy = np.meshgrid(
                np.arange(0.42, 0.581, 0.002),
                np.arange(0.42, 0.581, 0.002),
            )
            fine_xy = np.column_stack((gx.ravel(), gy.ravel()))
            fine_records = np.zeros(len(fine_xy), dtype())
            fine_records["x"], fine_records["y"] = fine_xy.T
            fine_records["z"] = 2.0
            fine_records["nx"] = 0.001
            fine_records["ny"] = -0.002
            fine_records["nz"] = np.sqrt(1.0 - 0.001 ** 2 - 0.002 ** 2)
            fine_records["scalar_ScanID"] = 999
            # Fine nongeometry values must not replace coarse donor data.
            fine_records["scalar_Intensity"] = 7777
            fine_records["scalar_Composite"] = 222
            fine_candidate = root / "fine-candidate.ply"
            write_ply(fine_candidate, fine_records)
            fine_archive = root / "fine-added.npz"
            np.savez_compressed(
                fine_archive,
                records=fine_records,
                candidate_xy=fine_xy,
                candidate_label=np.ones(len(fine_xy), np.int32),
            )
            fine_manifest = root / "fine-manifest.json"
            fine_manifest.write_text(json.dumps({
                "candidate_only": True,
                "canonical_install_performed": False,
                "existing_payload_byte_exact": True,
                "addition_count": len(fine_records),
                "archive": str(fine_archive),
                "archive_sha256": pipeline.sha256_path(fine_archive),
                "candidate": {
                    "path": str(fine_candidate),
                    "sha256": pipeline.sha256_path(fine_candidate),
                    "points": len(fine_records),
                },
                "config": {
                    "path": str(config),
                    "sha256": pipeline.sha256_path(config),
                },
                "reference_provenance": provenance,
                "review_bbox": [0.35, 0.65, 0.35, 0.65],
                "holes": [{
                    "seed_id": "test-hole",
                    "label": 1,
                    "accepted": True,
                    "bounds": [0.42, 0.58, 0.42, 0.58],
                }],
            }))

            output = root / "coarse.candidate.ply"
            manifest = root / "coarse-manifest.json"
            archive = root / "coarse-added.npz"

            def surface(x, y):
                return (
                    # Reproduce the former production drift: recomputing the
                    # analytic surface differs microscopically from archived
                    # fine float32 Z.
                    np.full(len(x), 2.0 + 9.54e-6),
                    np.tile([0.0, 0.0, 1.0], (len(x), 1)),
                )

            result = pipeline.run_coarse_hole_completion_from_fine(
                source_water_path=coarse_source,
                fine_manifest_path=fine_manifest,
                terrain_paths=[terrain],
                config_path=config,
                output_path=output,
                audit_path=manifest,
                archive_path=archive,
                spacing_m=0.005,
                reference_surface=surface,
                reference_provenance=provenance,
            )
            self.assertGreater(result.addition_count, 0)
            value = json.loads(manifest.read_text())
            self.assertTrue(
                value["cross_scale"]["coarse_xyz_exact_subset_of_fine_records_xyz"]
            )
            self.assertTrue(
                value["cross_scale"][
                    "coarse_normals_exact_subset_of_fine_records_normals"
                ]
            )
            self.assertTrue(
                value["cross_scale"][
                    "nongeometry_fields_preserved_from_coarse_donors"
                ]
            )
            self.assertEqual(
                value["cross_scale"]["geometry_fields_copied_from_fine_records"],
                ["x", "y", "z", "nx", "ny", "nz"],
            )
            with np.load(archive, allow_pickle=False) as stored:
                selected = stored["fine_selection_index"]
                self.assertEqual(len(selected), len(np.unique(selected)))
                self.assertTrue(np.all(stored["fine_component_label"] == 1))
                for name in ("x", "y", "z", "nx", "ny", "nz"):
                    self.assertEqual(
                        np.ascontiguousarray(stored["records"][name]).tobytes(),
                        np.ascontiguousarray(fine_records[name][selected]).tobytes(),
                        name,
                    )
                # The analytic surface really would have produced a distinct
                # float32 Z, proving the regression protects against micro-drift.
                self.assertFalse(np.array_equal(
                    stored["records"]["z"],
                    np.full(
                        len(selected), 2.0 + 9.54e-6,
                        dtype=stored["records"].dtype["z"],
                    ),
                ))
                self.assertTrue(
                    np.all(stored["records"]["scalar_Intensity"] == 5000)
                )
                self.assertTrue(
                    np.all(stored["records"]["scalar_Composite"] == 100)
                )


if __name__ == "__main__":
    unittest.main()
