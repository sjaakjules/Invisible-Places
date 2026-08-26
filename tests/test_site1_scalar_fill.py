import hashlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_scalar_fill.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("site1_scalar_fill", SCRIPT)
SCALAR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SCALAR
SPEC.loader.exec_module(SCALAR)


def synthetic_dtype():
    fields = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
        ("scalar_Intensity", "<f8"),
        ("scalar_Composite", "<f4"),
        ("scalar_ScanID", "<f4"),
    ]
    fields.extend((name, "<f4") for name in SCALAR.REPAIRABLE_FIELDS)
    return np.dtype(fields)


def populated_records(count):
    records = np.zeros(count, dtype=synthetic_dtype())
    records["red"] = 12
    records["green"] = 34
    records["blue"] = 56
    records["scalar_Intensity"] = np.arange(count, dtype=np.float64) + 5_000.125
    records["scalar_Composite"] = np.arange(count, dtype=np.float32) + 50.25
    records["scalar_ScanID"] = 7.0
    records["nz"] = 1.0
    for scale, multiplier in (("Fine", 1.0), ("Medium", 2.0), ("Broad", 3.0)):
        records[f"scalar_A_R_MeanCurvature_{scale}"] = 0.2 * multiplier
        records[f"scalar_A_R_CrossCurvature_{scale}"] = -0.1 * multiplier
        records[f"scalar_A_R_Recession_{scale}"] = 0.0002 * multiplier
        records[f"scalar_A_R_Roughness_{scale}"] = 0.002 * multiplier
    records["scalar_A_R_MeanCurvature_Combined"] = 0.1234567
    records["scalar_A_R_CrossCurvature_Combined"] = -0.1234567
    records["scalar_A_R_Recession_Combined"] = 0.25
    records["scalar_A_R_Roughness_Combined"] = 0.5
    records["scalar_A_R_RoughnessRelative_FineMedium"] = 0.5
    records["scalar_A_R_Shelter_Lower"] = 0.6
    records["scalar_A_R_RainExposure_Lower"] = 0.4
    records["scalar_A_R_SVF_Lower"] = 0.7
    records["scalar_A_R_Downhill_X"] = 0.2
    records["scalar_A_R_Downhill_Y"] = -0.3
    records["scalar_A_R_Downhill_Z"] = 0.0
    records["scalar_A_R_DownhillMagnitude"] = 0.4
    records["scalar_A_R_Horizontalness"] = 0.9
    records["scalar_A_R_Slope_deg"] = 20.0
    return records


def write_ply(path, records):
    with open(path, "wb") as handle:
        SCALAR.v6.write_ply_header(handle, records.dtype, len(records), ["synthetic test cloud"])
        records.tofile(handle)


def read_records(path):
    info = SCALAR.inspect_fixed_stride_ply(path)
    return np.array(
        np.memmap(path, dtype=info.dtype, mode="r", offset=info.offset, shape=(info.count,))
    )


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ScalarRepairTests(unittest.TestCase):
    def options(self, **overrides):
        values = {
            "chunk_size": 2,
            "max_donors_per_group": 100,
            "donor_query_k": 4,
            "workers": 1,
            "prefer_clone": False,
        }
        values.update(overrides)
        return SCALAR.RepairOptions(**values)

    def test_local_idw_recomputes_only_invalid_derived_and_preserves_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-SAND-5mm-source.ply"
            output = directory / "Site1-SAND-5mm-candidate.ply"
            records = populated_records(6)
            records["x"] = [0.0, 0.02, 0.01, 1.0, 1.8, 0.015]
            original_finite_combined = records["scalar_A_R_MeanCurvature_Combined"][0]
            fine_fields = SCALAR.COMPONENT_GROUPS["Fine"]
            for field in fine_fields:
                records[field][2:4] = np.nan
            records["scalar_A_R_MeanCurvature_Fine"][:2] = [0.1, 0.3]
            records["scalar_A_R_MeanCurvature_Combined"][2:4] = np.nan
            records["scalar_A_R_RoughnessRelative_FineMedium"][2:4] = np.nan
            for field in SCALAR.COMPONENT_GROUPS["Directional"]:
                records[field][2] = np.nan
            # Partial corruption must fill only the one invalid component.
            records["scalar_A_R_Roughness_Fine"][5] = np.nan
            finite_partial_companion = records["scalar_A_R_Recession_Fine"][5].tobytes()
            # A finite negative zero exercises bit-level preservation.
            records["scalar_A_R_CrossCurvature_Fine"][4] = -0.0
            write_ply(source, records)
            source_hash = sha256(source)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="SAND",
                spacing="5mm",
                options=self.options(),
            )
            repaired = read_records(output)

            self.assertEqual(sha256(source), source_hash)
            self.assertTrue(report["verification"]["verified"])
            self.assertAlmostEqual(
                repaired["scalar_A_R_MeanCurvature_Fine"][2], 0.2, delta=1e-6
            )
            normalization = report["derived"]["combination"]["inference"][
                "values"
            ]["MeanCurvature"]
            expected_combined = sum(
                weight
                * np.clip(
                    repaired[f"scalar_A_R_MeanCurvature_{scale}"][2]
                    / normalization[scale],
                    -1.0,
                    1.0,
                )
                for scale, weight in zip(
                    SCALAR.DERIVED_SCALES,
                    SCALAR.DERIVED_WEIGHTS,
                )
            )
            self.assertAlmostEqual(
                repaired["scalar_A_R_MeanCurvature_Combined"][2],
                expected_combined,
                delta=1e-6,
            )
            self.assertEqual(
                repaired["scalar_A_R_MeanCurvature_Combined"][0],
                original_finite_combined,
            )
            self.assertTrue(np.isnan(repaired["scalar_A_R_MeanCurvature_Fine"][3]))
            self.assertTrue(
                np.isfinite(repaired["scalar_A_R_MeanCurvature_Combined"][3])
            )
            self.assertTrue(np.isfinite(repaired["scalar_A_R_Downhill_X"][2]))
            self.assertTrue(np.isfinite(repaired["scalar_A_R_Roughness_Fine"][5]))
            self.assertEqual(
                repaired["scalar_A_R_Recession_Fine"][5].tobytes(),
                finite_partial_companion,
            )
            self.assertEqual(
                repaired["scalar_Intensity"].tobytes(), records["scalar_Intensity"].tobytes()
            )
            self.assertEqual(
                repaired["scalar_A_R_CrossCurvature_Fine"][4].tobytes(),
                records["scalar_A_R_CrossCurvature_Fine"][4].tobytes(),
            )
            self.assertEqual(report["groups"]["Fine"]["filled_rows_by_bucket"]["le_25mm"], 2)
            self.assertEqual(report["groups"]["Fine"]["filled_by_field"][fine_fields[0]], 1)
            self.assertNotIn(
                "scalar_A_R_Downhill_X", report["joint_complete_donor_bundle"]
            )
            self.assertEqual(
                report["groups"]["Directional"]["direct_from_normals"][
                    "filled_rows"
                ],
                1,
            )
            self.assertEqual(
                report["derived"]["combination"]["algorithm"],
                SCALAR.DERIVED_COMBINATION_ALGORITHM,
            )

    def test_cleanmesh_p95_combination_matches_reference_with_outliers(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-ROCK-5mm-source.ply"
            output = directory / "Site1-ROCK-5mm-derived-candidate.ply"
            records = populated_records(101)
            rank = np.arange(1, 102, dtype=np.float32)
            factors = {
                "MeanCurvature": (0.1, 0.2, 0.4),
                "CrossCurvature": (-0.15, -0.3, -0.6),
                "Recession": (0.0001, 0.0002, 0.0004),
                "Roughness": (0.0002, 0.0004, 0.0008),
            }
            for metric, scales in factors.items():
                for scale, factor in zip(SCALAR.DERIVED_SCALES, scales):
                    records[f"scalar_A_R_{metric}_{scale}"] = rank * factor
                    records[f"scalar_A_R_{metric}_{scale}"][-1] *= 10_000.0
            for field in SCALAR.DERIVED_FIELDS:
                records[field] = np.nan
            write_ply(source, records)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="ROCK",
                spacing="5mm",
                options=self.options(
                    chunk_size=17,
                    derived_normalization_sample_limit=1000,
                ),
            )
            repaired = read_records(output)
            inferred = report["derived"]["combination"]["inference"]
            max_error = 0.0
            for metric, scales in factors.items():
                expected_normalization = {
                    scale: abs(factor) * 96.0
                    for scale, factor in zip(SCALAR.DERIVED_SCALES, scales)
                }
                for scale in SCALAR.DERIVED_SCALES:
                    self.assertAlmostEqual(
                        inferred["values"][metric][scale],
                        expected_normalization[scale],
                        delta=3.0e-6,
                    )
                expected = np.minimum(rank / 96.0, 1.0)
                if metric == "CrossCurvature":
                    expected = -expected
                actual = repaired[f"scalar_A_R_{metric}_Combined"]
                max_error = max(
                    max_error,
                    float(np.max(np.abs(actual - expected))),
                )
            self.assertLess(max_error, 2.0e-7)
            np.testing.assert_allclose(
                repaired["scalar_A_R_RoughnessRelative_FineMedium"],
                0.5,
                rtol=0.0,
                atol=1.0e-7,
            )
            self.assertEqual(
                inferred["sampled_count_by_field"][
                    "scalar_A_R_MeanCurvature_Fine"
                ],
                101,
            )
            self.assertTrue(report["verification"]["verified"])

    def test_cleanmesh_combination_renormalizes_over_finite_scales(self):
        records = populated_records(2)
        records["scalar_A_R_MeanCurvature_Fine"] = [0.5, np.nan]
        records["scalar_A_R_MeanCurvature_Medium"] = [np.nan, np.nan]
        records["scalar_A_R_MeanCurvature_Broad"] = [1.0, np.nan]
        normalization = {
            metric: {scale: 1.0 for scale in SCALAR.DERIVED_SCALES}
            for metric in SCALAR.METRICS
        }
        values, valid = SCALAR._derived_value(
            "scalar_A_R_MeanCurvature_Combined",
            records,
            normalization,
            SCALAR.DERIVED_WEIGHTS,
        )
        expected = (0.45 * 0.5 + 0.20 * 1.0) / (0.45 + 0.20)
        self.assertAlmostEqual(values[0], expected, places=12)
        self.assertTrue(valid[0])
        self.assertFalse(valid[1])

    def test_directional_invalid_values_are_derived_from_upward_normal(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-SAND-5mm-source.ply"
            output = directory / "Site1-SAND-5mm-direction-candidate.ply"
            records = populated_records(1)
            records["nx"] = 0.6
            records["ny"] = 0.0
            records["nz"] = -0.8
            finite_y = np.float32(0.125)
            for field in SCALAR.COMPONENT_GROUPS["Directional"]:
                records[field] = np.nan
            records["scalar_A_R_Downhill_Y"] = finite_y
            write_ply(source, records)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="SAND",
                spacing="5mm",
                options=self.options(),
            )
            repaired = read_records(output)

            self.assertAlmostEqual(repaired["scalar_A_R_Downhill_X"][0], -1.0)
            self.assertEqual(
                repaired["scalar_A_R_Downhill_Y"][0].tobytes(),
                finite_y.tobytes(),
            )
            self.assertEqual(repaired["scalar_A_R_Downhill_Z"][0], 0.0)
            self.assertAlmostEqual(
                repaired["scalar_A_R_DownhillMagnitude"][0], 0.75, places=6
            )
            self.assertAlmostEqual(
                repaired["scalar_A_R_Horizontalness"][0], 0.8, places=6
            )
            self.assertAlmostEqual(
                repaired["scalar_A_R_Slope_deg"][0], 36.869897, places=5
            )
            direct = report["groups"]["Directional"]["direct_from_normals"]
            self.assertEqual(direct["filled_rows"], 1)
            self.assertEqual(direct["flipped_upward_rows"], 1)
            self.assertEqual(report["joint_complete_donor_bundle"], [])
            self.assertTrue(report["verification"]["verified"])

    def test_group_specific_xy_fallback_fills_only_remaining_group_values(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-ROCK-5mm-source.ply"
            output = directory / "Site1-ROCK-5mm-xy-candidate.ply"
            records = populated_records(3)
            records["x"] = [0.0, 0.1, 5.0]
            records["z"] = [0.0, 0.0, 100.0]
            # There is deliberately no source row complete for both active
            # groups: row 0 can donate Fine only and row 1 Broad only.
            for field in SCALAR.COMPONENT_GROUPS["Fine"]:
                records[field][1:] = np.nan
            for field in SCALAR.COMPONENT_GROUPS["Broad"]:
                records[field][[0, 2]] = np.nan
            finite_medium = records["scalar_A_R_MeanCurvature_Medium"][2].tobytes()
            write_ply(source, records)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="ROCK",
                spacing="5mm",
                options=self.options(
                    distance_buckets_m=(0.4,),
                    xy_fallback_buckets_m=(1.0, 10.0),
                ),
            )
            repaired = read_records(output)

            self.assertTrue(
                np.all(
                    np.isfinite(
                        [
                            repaired[field][1:]
                            for field in SCALAR.COMPONENT_GROUPS["Fine"]
                        ]
                    )
                )
            )
            self.assertEqual(
                repaired["scalar_A_R_MeanCurvature_Medium"][2].tobytes(),
                finite_medium,
            )
            fine = report["groups"]["Fine"]
            self.assertEqual(fine["filled_rows_by_bucket"]["le_400mm"], 0)
            self.assertEqual(
                fine["xy_fallback"]["filled_rows_by_bucket"]["le_10000mm"],
                1,
            )
            self.assertEqual(fine["remaining_candidate_rows"], 0)
            self.assertEqual(fine["xy_fallback"]["donors"]["source"], 1)
            self.assertEqual(
                report["groups"]["Broad"]["xy_fallback"]["donors"]["source"],
                1,
            )
            self.assertEqual(fine["donors"]["source"], 0)
            self.assertGreater(
                fine["xy_fallback"][
                    "nearest_abs_z_separation_sample_quantiles_m"
                ]["max"],
                99.0,
            )
            self.assertTrue(report["verification"]["verified"])

    def test_zero_normal_directional_row_uses_group_specific_3d_before_xy(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-VEG-5mm-source.ply"
            output = directory / "Site1-VEG-5mm-direction-candidate.ply"
            records = populated_records(2)
            records["x"] = [0.0, 0.01]
            records["nx"][1] = 0.0
            records["ny"][1] = 0.0
            records["nz"][1] = 0.0
            for field in SCALAR.COMPONENT_GROUPS["Directional"]:
                records[field][1] = np.nan
            write_ply(source, records)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="VEG",
                spacing="5mm",
                options=self.options(xy_fallback_buckets_m=(0.4, 10.0)),
            )
            repaired = read_records(output)

            directional = report["groups"]["Directional"]
            self.assertEqual(
                directional["direct_from_normals"]["zero_length_normal_rows"],
                1,
            )
            self.assertEqual(
                directional["group_specific_3d_after_normals"][
                    "filled_rows_by_bucket"
                ]["le_25mm"],
                1,
            )
            self.assertEqual(directional["remaining_candidate_rows"], 0)
            self.assertNotIn("xy_fallback", directional)
            self.assertTrue(
                np.all(
                    np.isfinite(
                        [
                            repaired[field][1]
                            for field in SCALAR.COMPONENT_GROUPS["Directional"]
                        ]
                    )
                )
            )
            self.assertTrue(report["verification"]["verified"])

    def test_repaired_5mm_same_role_can_supply_1mm_donors(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-ROCK-1mm-source.ply"
            fallback = directory / "Site1-ROCK-5mm-repaired.ply"
            output = directory / "Site1-ROCK-1mm-candidate.ply"
            one_mm = populated_records(1)
            one_mm["x"] = 0.0
            for field in SCALAR.COMPONENT_GROUPS["Broad"]:
                one_mm[field] = np.nan
            one_mm["scalar_A_R_MeanCurvature_Combined"] = np.nan
            five_mm = populated_records(1)
            five_mm["x"] = 0.01
            five_mm["scalar_A_R_MeanCurvature_Broad"] = 1.25
            write_ply(source, one_mm)
            write_ply(fallback, five_mm)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="ROCK",
                spacing="1mm",
                fallback_5mm_path=fallback,
                options=self.options(),
            )
            repaired = read_records(output)

            self.assertEqual(repaired["scalar_A_R_MeanCurvature_Broad"][0], 1.25)
            self.assertTrue(np.isfinite(repaired["scalar_A_R_MeanCurvature_Combined"][0]))
            self.assertEqual(report["groups"]["Broad"]["donors"]["source"], 0)
            self.assertEqual(report["groups"]["Broad"]["donors"]["fallback"], 1)
            self.assertEqual(
                report["groups"]["Broad"]["nearest_origin_for_filled_rows"]["fallback"], 1
            )

    def test_out_of_bound_complete_group_is_not_a_donor(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-VEG-1mm-source.ply"
            fallback = directory / "Site1-VEG-5mm-repaired.ply"
            output = directory / "Site1-VEG-1mm-candidate.ply"
            target = populated_records(1)
            for field in SCALAR.COMPONENT_GROUPS["Fine"]:
                target[field] = np.nan
            donor = populated_records(1)
            donor["x"] = 0.01
            donor["scalar_A_R_MeanCurvature_Fine"] = 50_000.0
            write_ply(source, target)
            write_ply(fallback, donor)

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="VEG",
                spacing="1mm",
                fallback_5mm_path=fallback,
                options=self.options(),
            )
            repaired = read_records(output)

            self.assertTrue(np.isnan(repaired["scalar_A_R_MeanCurvature_Fine"][0]))
            self.assertEqual(report["groups"]["Fine"]["donors"]["source"], 0)
            self.assertEqual(report["groups"]["Fine"]["donors"]["fallback"], 0)
            self.assertTrue(report["verification"]["verified"])

    def test_cross_role_fallback_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-SAND-1mm-source.ply"
            fallback = directory / "Site1-ROCK-5mm-repaired.ply"
            output = directory / "candidate.ply"
            write_ply(source, populated_records(1))
            write_ply(fallback, populated_records(1))

            with self.assertRaisesRegex(RuntimeError, "same-role"):
                SCALAR.repair_scalar_file(
                    source,
                    output,
                    role="SAND",
                    spacing="1mm",
                    fallback_5mm_path=fallback,
                    options=self.options(),
                )

    def test_complete_groups_are_skipped_without_a_donor_catalogue(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "Site1-SAND-5mm-source.ply"
            output = directory / "Site1-SAND-5mm-complete-candidate.ply"
            write_ply(source, populated_records(3))

            report = SCALAR.repair_scalar_file(
                source,
                output,
                role="SAND",
                spacing="5mm",
                options=self.options(),
            )

            self.assertEqual(report["joint_complete_donor_bundle"], [])
            self.assertTrue(
                all(
                    group["skipped"] == "no_nonfinite_values"
                    for group in report["groups"].values()
                )
            )
            self.assertEqual(Path(source).read_bytes(), Path(output).read_bytes())

    def test_canonical_output_name_is_rejected(self):
        canonical = SCALAR.DEFAULT_DATA_DIR / "Site1-SAND-5mm.ply"
        staged = SCALAR.DEFAULT_DATA_DIR / "PatchRefinement/run/Site1-SAND-5mm.ply"

        self.assertTrue(SCALAR.is_canonical_output(canonical))
        self.assertFalse(SCALAR.is_canonical_output(staged))


if __name__ == "__main__":
    unittest.main()
