import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock

import numpy as np

from tests.test_rebuild_site1_fossils_v10 import V10


class VerifyDispatchTests(unittest.TestCase):
    def test_verify_passes_config_and_manifest_noise_scale_to_both_audits(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            dense = run / "dense.ply"
            coarse = run / "coarse.ply"
            manifest = {
                "source_snapshots": {
                    "data_dir": str(run / "data"),
                    "v9_run": str(run / "v9"),
                    "config": str(run / "config.json"),
                },
                "dense_candidate": {
                    "path": str(dense), "sha256": "dense-hash"
                },
                "coarse_candidate": {
                    "path": str(coarse), "sha256": "coarse-hash"
                },
                "noise": {"scale": 0.85},
            }
            (run / "manifest.json").write_text(json.dumps(manifest))
            config = {"acceptance": {}}

            def audit(path, *_args, **_kwargs):
                return {
                    "failures": [],
                    "sha256": (
                        "dense-hash" if path == dense else "coarse-hash"
                    ),
                }

            with (
                mock.patch.object(V10, "load_config", return_value=config),
                mock.patch.object(V10, "load_surface_reference", return_value=object()),
                mock.patch.object(V10, "audit_candidate", side_effect=audit) as audited,
                mock.patch.object(
                    V10,
                    "verify_pipeline_provenance",
                    return_value={"verified": True, "failures": []},
                ),
                mock.patch.object(V10, "require_runtime_dependencies"),
            ):
                report = V10.verify(
                    SimpleNamespace(run_dir=run), acquire_lock=False
                )

            self.assertTrue(report["verified"])
            self.assertEqual(audited.call_count, 2)
            for call in audited.call_args_list:
                self.assertIs(call.args[4], config)
                self.assertEqual(call.kwargs["noise_scale"], 0.85)


class PipelineProvenanceTests(unittest.TestCase):
    def test_valid_small_manifest_chain_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            data = run / "snapshots/data"
            v9 = run / "snapshots/v9"
            data.mkdir(parents=True)
            v9.mkdir(parents=True)
            dtype = np.dtype([("x", "<f4")])

            def tiny_ply(path):
                with open(path, "wb") as handle:
                    V10.v6.write_ply_header(handle, dtype, 1, ["fixture"])
                    np.zeros(1, dtype).tofile(handle)

            terrain = {}
            snapshot_paths = []
            for role in ("SAND", "ROCK"):
                for spacing in ("1mm", "5mm"):
                    path = data / f"Site1-{role}-{spacing}.ply"
                    tiny_ply(path)
                    terrain[f"{role}-{spacing}"] = V10.file_fingerprint(path)
                    snapshot_paths.append(path)
            water = data / "Site1-WATER-5mm.ply"
            candidate = v9 / "v9-candidate.ply"
            tiny_ply(water)
            tiny_ply(candidate)
            surface = v9 / "surface-v9.npz"
            surface.write_bytes(b"fixture-npz")
            config_path = run / "snapshots/config.json"
            config_path.write_text("{}")
            snapshot_paths.extend((water, surface, candidate, config_path))
            inputs = {
                "algorithm": "fixture",
                "terrain_sources": terrain,
                "source_water": V10.file_fingerprint(water),
                "v9_surface": V10.file_fingerprint(surface),
                "v9_candidate": V10.file_fingerprint(candidate),
                "config": V10.file_fingerprint(config_path),
            }
            signature = V10._json_sha256(inputs)
            state = {
                "schema_version": V10.BUILD_STATE_SCHEMA,
                "input_signature": signature,
                "inputs": inputs,
                "stages": {
                    "source_snapshots": {
                        "outputs": [
                            V10.file_fingerprint(path)
                            for path in snapshot_paths
                        ]
                    }
                },
            }
            (run / "build-state.json").write_text(json.dumps(state))

            downsample = {}
            blockers = {}
            for label, spacing, kept in (
                ("2mm", 0.002, 12), ("5mm", 0.005, 4)
            ):
                path = run / f"downsample-{label}.json"
                path.write_text(json.dumps({
                    "method": "greedy_spatial_minimum_distance",
                    "minimum_spacing_m": spacing,
                    "priority_scan_id": int(V10.BASE_PRIORITY_SCAN_ID),
                    "priority_minimum_spacing_m": spacing,
                    "priority_output_points": 3,
                    "output_points": kept + 3,
                    "non_finite_positions": 0,
                }))
                downsample[label] = V10.file_fingerprint(path)
                blockers[label] = {
                    "minimum_spacing_m": spacing,
                    "source_water_points": kept,
                    "kept_water_points": kept,
                    "output_total_points": kept + 3,
                }

            analysis_path = run / "analysis.json"
            analysis_path.write_text(json.dumps({
                "input_points": 15,
                "base_voxels": 10,
                "valid_proxy_voxels": {"fine": 9, "medium": 8},
            }))
            scalar_path = run / "scalar.json"
            scalar_path.write_text(json.dumps({
                "verification": {"verified": True}
            }))
            final_path = run / "final.json"
            final_path.write_text(json.dumps({"verified": True}))
            manifest = {
                "input_signature": signature,
                "source_snapshots": {
                    "data_dir": str(data),
                    "v9_run": str(v9),
                    "v9_candidate": str(candidate),
                    "config": str(config_path),
                },
                "downsample_reports": downsample,
                "all_terrain_blocker_reports": blockers,
                "dense_candidate": {"points": 12},
                "coarse_candidate": {"points": 4},
                "analysis_report": V10.file_fingerprint(analysis_path),
                "scalar_repair_report": V10.file_fingerprint(scalar_path),
                "final_scalar_verification": V10.file_fingerprint(final_path),
            }
            config = {"acceptance": {
                "minimum_dense_spacing_m": 0.002,
                "minimum_coarse_spacing_m": 0.005,
            }}
            report = V10.verify_pipeline_provenance(
                run, manifest, config
            )
            self.assertTrue(report["verified"], report["failures"])


if __name__ == "__main__":
    unittest.main()
