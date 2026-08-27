import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/site1_v11_water_density.py"
SPEC = importlib.util.spec_from_file_location("site1_v11_water_density", SCRIPT)
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)


def grid_points(x_values, y_values):
    x, y = np.meshgrid(np.asarray(x_values), np.asarray(y_values))
    return np.column_stack((x.ravel(), y.ravel()))


PLY_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("scalar_Intensity", "<f4"),
        ("scalar_ScanID", "<f4"),
    ]
)


def records_for_xy(xy, *, scan_id=999.0, offset=0.0):
    xy = np.asarray(xy, np.float64)
    record = np.zeros(len(xy), PLY_DTYPE)
    record["x"] = xy[:, 0]
    record["y"] = xy[:, 1]
    record["z"] = np.arange(len(xy), dtype=np.float32) + offset
    record["scalar_Intensity"] = (
        5000.0 + offset + np.arange(len(xy), dtype=np.float32)
    )
    record["scalar_ScanID"] = scan_id
    return record


def write_fixture(path, records):
    typemap = {
        "f4": "float",
        "f8": "double",
        "u1": "uchar",
        "i1": "char",
        "i2": "short",
        "u2": "ushort",
        "i4": "int",
        "u4": "uint",
    }
    with Path(path).open("wb") as handle:
        handle.write(b"ply\n")
        handle.write(b"format binary_little_endian 1.0\n")
        handle.write(f"element vertex {len(records)}\n".encode("ascii"))
        for name in records.dtype.names:
            code = records.dtype[name].str.lstrip("<>=|")
            handle.write(f"property {typemap[code]} {name}\n".encode("ascii"))
        handle.write(b"end_header\n")
        records.tofile(handle)


def read_fixture(path):
    info = V11.inspect_fixed_stride_ply(path)
    mm = np.memmap(
        path,
        dtype=info.dtype,
        mode="r",
        offset=info.offset,
        shape=(info.count,),
    )
    result = np.asarray(mm).copy()
    del mm
    return result


class GeometryMaskTests(unittest.TestCase):
    def test_bbox_and_polygon_have_explicit_boundary_policies(self):
        points = np.array(
            [
                [0.0, 0.0],
                [0.5, 0.5],
                [1.0, 0.5],
                [1.1, 0.5],
                [-0.1, 0.2],
            ]
        )
        polygon = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        np.testing.assert_array_equal(
            V11.bbox_mask(points, [0.0, 1.0, 0.0, 1.0]),
            [True, True, True, False, False],
        )
        np.testing.assert_array_equal(
            V11.bbox_mask(points, [0.0, 1.0, 0.0, 1.0], inclusive=False),
            [False, True, False, False, False],
        )
        np.testing.assert_array_equal(
            V11.polygon_mask(points, polygon),
            [True, True, True, False, False],
        )
        np.testing.assert_array_equal(
            V11.polygon_mask(points, polygon, include_boundary=False),
            [False, True, False, False, False],
        )

    def test_invalid_self_evident_geometry_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "xmin < xmax"):
            V11.bbox_mask([[0.0, 0.0]], [1.0, 0.0, 0.0, 1.0])
        with self.assertRaisesRegex(ValueError, "at least three"):
            V11.polygon_mask([[0.0, 0.0]], [[0.0, 0.0], [1.0, 0.0]])


class MeasuredDensityTests(unittest.TestCase):
    def test_measured_density_distinguishes_fine_and_coarse_support(self):
        fine = grid_points(np.arange(-1.0, 1.01, 0.10), np.arange(-1.0, 1.01, 0.10))
        coarse = grid_points(np.arange(-1.0, 1.01, 0.20), np.arange(-1.0, 1.01, 0.20))
        query = np.array([[0.0, 0.0]])
        fine_result = V11.measure_local_2d_density(query, fine, neighbours=12)
        coarse_result = V11.measure_local_2d_density(query, coarse, neighbours=12)
        self.assertGreater(fine_result.density_per_m2[0], coarse_result.density_per_m2[0])
        self.assertLess(fine_result.equivalent_spacing_m[0], coarse_result.equivalent_spacing_m[0])
        self.assertAlmostEqual(
            coarse_result.equivalent_spacing_m[0]
            / fine_result.equivalent_spacing_m[0],
            2.0,
            places=8,
        )
        self.assertAlmostEqual(fine_result.nearest_spacing_m[0], 0.10, places=8)

    def test_exact_query_point_is_not_counted_as_its_own_neighbour(self):
        support = np.array([[0.0, 0.0], [0.25, 0.0], [0.5, 0.0]])
        result = V11.measure_local_2d_density(support[:1], support, neighbours=2)
        self.assertEqual(result.neighbour_count[0], 2)
        self.assertAlmostEqual(result.nearest_spacing_m[0], 0.25)


class EastTaperTests(unittest.TestCase):
    def test_vertical_guide_has_signed_east_distance_and_c1_taper(self):
        x = np.array([-0.2, 0.0, 1e-6, 0.5, 1.0 - 1e-6, 1.0, 1.2])
        points = np.column_stack((x, np.zeros_like(x)))
        guide = np.array([[0.0, -2.0], [0.0, 2.0]])
        result = V11.c1_east_taper(
            points, guide, start_m=0.0, end_m=1.0, floor_ratio=0.2
        )
        np.testing.assert_allclose(result.signed_distance_m, x)
        self.assertTrue(np.all(np.diff(result.factor) <= 0.0))
        self.assertAlmostEqual(result.factor[0], 1.0)
        self.assertAlmostEqual(result.factor[3], 0.6)
        self.assertAlmostEqual(result.factor[-1], 0.2)
        self.assertLess(1.0 - result.factor[2], 1e-10)
        self.assertLess(result.factor[4] - 0.2, 1e-10)

    def test_density_taper_increases_spacing_by_inverse_square_root(self):
        result = V11.spacing_for_density_taper(
            [0.002, 0.002, 0.002], [1.0, 0.25, 0.04]
        )
        np.testing.assert_allclose(result, [0.002, 0.004, 0.010])


class PointwiseSelectionTests(unittest.TestCase):
    def test_thinning_is_deterministic_pointwise_and_permutation_stable(self):
        rng = np.random.default_rng(19)
        points = rng.uniform(-1.0, 1.0, size=(250, 2))
        radius = 0.08 + 0.04 * (points[:, 0] + 1.0) / 2.0
        first = V11.deterministic_pointwise_thinning(points, radius, seed=77)
        repeat = V11.deterministic_pointwise_thinning(points, radius, seed=77)
        np.testing.assert_array_equal(first.selected_indices, repeat.selected_indices)

        permutation = rng.permutation(len(points))
        second = V11.deterministic_pointwise_thinning(
            points[permutation], radius[permutation], seed=77
        )
        first_points = {tuple(row) for row in points[first.selected_indices]}
        second_points = {
            tuple(row) for row in points[permutation][second.selected_indices]
        }
        self.assertEqual(first_points, second_points)
        for offset, index in enumerate(first.selected_indices):
            later = first.selected_indices[offset + 1 :]
            if not len(later):
                continue
            distance = np.linalg.norm(points[later] - points[index], axis=1)
            required = np.maximum(radius[later], radius[index])
            self.assertTrue(np.all(distance + 1e-12 >= required))

    def test_additions_respect_both_terrain_and_existing_water(self):
        terrain = np.array([[0.0, 0.0]])
        water = np.array([[1.0, 0.0]])
        candidates = np.array(
            [[0.05, 0.0], [0.30, 0.0], [0.46, 0.0], [0.70, 0.0], [0.95, 0.0]]
        )
        radius = np.full(len(candidates), 0.20)
        result = V11.continuous_variable_radius_additions(
            candidates,
            radius,
            terrain_xy=terrain,
            water_xy=water,
            seed=91,
        )
        selected = candidates[result.selected_indices]
        self.assertTrue(np.all(np.linalg.norm(selected - terrain[0], axis=1) >= 0.20))
        self.assertTrue(np.all(np.linalg.norm(selected - water[0], axis=1) >= 0.20))
        self.assertNotIn(0, result.selected_indices)
        self.assertNotIn(4, result.selected_indices)

    def test_refinement_keeps_fixed_existing_before_using_reservoir(self):
        existing = np.array([[0.0, 0.0], [0.10, 0.0], [0.20, 0.0], [1.0, 0.0]])
        reservoir = np.array([[0.05, 0.0], [0.15, 0.0], [0.50, 0.0], [0.80, 0.0]])
        terrain = np.array([[0.0, 0.3], [0.3, 0.3], [0.6, 0.3], [0.9, 0.3]])
        plan = V11.plan_water_density_refinement(
            existing,
            reservoir,
            terrain,
            existing_refine_mask=[True, True, True, False],
            neighbours=3,
            minimum_spacing_m=0.10,
            maximum_base_spacing_m=0.10,
            maximum_tapered_spacing_m=0.10,
            seed=33,
        )
        self.assertIn(3, plan.existing_selected_indices)
        accepted = np.concatenate(
            (terrain, existing[plan.existing_selected_indices]), axis=0
        )
        additions = reservoir[plan.reservoir_selected_indices]
        if len(additions):
            distance = np.linalg.norm(
                additions[:, None, :] - accepted[None, :, :], axis=2
            )
            self.assertTrue(np.all(np.min(distance, axis=1) >= 0.10 - 1e-12))


class InterfaceAuditTests(unittest.TestCase):
    def test_audit_accepts_continuous_density_and_rejects_sparse_water(self):
        terrain = grid_points(
            np.arange(-0.50, 0.001, 0.05), np.arange(-0.50, 0.501, 0.05)
        )
        water = grid_points(
            np.arange(0.05, 0.501, 0.05), np.arange(-0.50, 0.501, 0.05)
        )
        interface = np.column_stack(
            (np.zeros(9), np.linspace(-0.30, 0.30, 9))
        )
        continuous = V11.audit_interface_density(
            interface,
            terrain,
            water,
            probe_offset_m=0.05,
            neighbours=8,
            maximum_median_ratio=1.25,
            maximum_p90_ratio=1.50,
        )
        self.assertTrue(continuous.passed)

        sparse_water = grid_points(
            np.arange(0.15, 0.501, 0.15), np.arange(-0.45, 0.451, 0.15)
        )
        sparse = V11.audit_interface_density(
            interface,
            terrain,
            sparse_water,
            probe_offset_m=0.10,
            neighbours=8,
            maximum_median_ratio=1.25,
            maximum_p90_ratio=1.50,
        )
        self.assertFalse(sparse.passed)
        self.assertGreater(sparse.median_spacing_ratio, 1.25)


class StreamingPlyTests(unittest.TestCase):
    def test_collect_filters_records_without_loading_unrelated_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reservoir.ply"
            records = records_for_xy(
                [[0.0, 0.0], [0.5, 0.5], [0.7, 0.7], [2.0, 2.0]]
            )
            records["scalar_ScanID"][2] = -987654.0
            write_fixture(path, records)
            selected = V11.collect_ply_xy(
                path,
                bbox=[0.25, 1.0, 0.25, 1.0],
                field_equals={"scalar_ScanID": 999.0},
                chunk_size=2,
            )
            np.testing.assert_array_equal(selected.indices, [1])
            np.testing.assert_allclose(selected.xy, [[0.5, 0.5]])

    def test_near_surface_filter_excludes_reflections_at_the_same_xy(self):
        records = records_for_xy([[1.0, 2.0], [1.0, 2.0], [1.0, 2.0]])
        records["z"] = [-0.50, 2.00, 2.04]
        terrain_filter = V11.make_near_surface_record_filter(
            lambda x, y: np.full(len(x), 2.0), below_m=0.015, above_m=0.025
        )
        np.testing.assert_array_equal(
            terrain_filter(records), [False, True, False]
        )

    def test_candidate_writer_preserves_selected_record_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            existing_path = root / "existing.ply"
            reservoir_path = root / "reservoir.ply"
            existing = records_for_xy(
                [[0.0, 0.0], [0.1, 0.0], [0.2, 0.0], [0.3, 0.0]], offset=10
            )
            reservoir = records_for_xy(
                [[1.0, 0.0], [1.1, 0.0], [1.2, 0.0]], offset=100
            )
            write_fixture(existing_path, existing)
            write_fixture(reservoir_path, reservoir)
            existing_selection = V11.make_ply_record_selection(
                existing_path,
                keep_by_default=True,
                indices=[1],
                label="existing",
            )
            reservoir_selection = V11.make_ply_record_selection(
                reservoir_path,
                keep_by_default=False,
                indices=[0, 2],
                label="reservoir",
            )
            output = root / "Site1-WATER-2mm.candidate.ply"
            count = V11.write_candidate_ply(
                output,
                [existing_selection, reservoir_selection],
                comments=["unit-test candidate"],
                chunk_size=2,
            )
            expected = np.concatenate((existing[[0, 2, 3]], reservoir[[0, 2]]))
            actual = read_fixture(output)
            self.assertEqual(count, len(expected))
            self.assertEqual(actual.tobytes(), expected.tobytes())

    def test_writer_refuses_canonical_and_source_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.ply"
            write_fixture(source, records_for_xy([[0.0, 0.0]]))
            selection = V11.make_ply_record_selection(
                source, keep_by_default=True, indices=[], label="source"
            )
            with self.assertRaisesRegex(ValueError, "canonical"):
                V11.write_candidate_ply(
                    root / "Site1-WATER-5mm.ply", [selection]
                )
            with self.assertRaisesRegex(ValueError, "canonical"):
                V11.write_candidate_ply(
                    root / "Site1-ROCK-1mm.ply", [selection]
                )
            with self.assertRaisesRegex(ValueError, "protected"):
                V11.write_candidate_ply(source, [selection], overwrite=True)

    def test_streaming_plan_reuses_only_water_reservoir_records(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            existing_path = root / "existing.ply"
            reservoir_path = root / "combined-v10-oversample.ply"
            terrain_path = root / "terrain.ply"

            existing_xy = grid_points(np.arange(0.0, 0.61, 0.10), [0.0, 0.10])
            existing = records_for_xy(existing_xy, offset=10)
            reservoir_xy = grid_points(np.arange(0.025, 0.601, 0.05), [0.025, 0.075])
            reservoir = records_for_xy(reservoir_xy, offset=100)
            reservoir["scalar_ScanID"][-1] = -987654.0
            terrain_xy = grid_points(np.arange(0.0, 0.61, 0.10), [0.20, 0.30])
            terrain = records_for_xy(terrain_xy, scan_id=4.0, offset=200)
            write_fixture(existing_path, existing)
            write_fixture(reservoir_path, reservoir)
            write_fixture(terrain_path, terrain)

            plan = V11.plan_streaming_water_candidate(
                existing_path,
                reservoir_path,
                [terrain_path],
                review_bbox=[0.0, 0.60, -0.01, 0.15],
                guide_xy=[[0.40, -1.0], [0.40, 1.0]],
                support_margin_m=0.20,
                taper_end_m=0.20,
                taper_floor_ratio=0.25,
                neighbours=4,
                minimum_spacing_m=0.05,
                maximum_base_spacing_m=0.10,
                maximum_tapered_spacing_m=0.20,
                terrain_support_is_prevalidated=True,
                chunk_size=5,
            )
            self.assertEqual(
                len(plan.reservoir_records.indices), len(reservoir) - 1
            )
            self.assertGreater(plan.terrain_record_count, 0)
            self.assertTrue(plan.output_selections[0].keep_by_default)
            self.assertFalse(plan.output_selections[1].keep_by_default)

            output = root / "Site1-WATER-2mm.refined.candidate.ply"
            written = V11.write_candidate_ply(
                output, plan.output_selections, chunk_size=4
            )
            self.assertEqual(
                written,
                sum(selection.selected_count for selection in plan.output_selections),
            )
            result = read_fixture(output)
            self.assertTrue(np.all(result["scalar_ScanID"] == 999.0))
            source_bytes = {
                row.tobytes() for row in np.concatenate((existing, reservoir[:-1]))
            }
            self.assertTrue(all(row.tobytes() in source_bytes for row in result))

    def test_split_writer_preserves_realistic_different_v10_schemas(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            existing_path = root / "full-v10-candidate.ply"
            reservoir_path = root / "short-v10-oversample.ply"
            terrain_path = root / "terrain.ply"
            existing = records_for_xy(
                grid_points(np.arange(0.0, 0.41, 0.10), [0.0, 0.10]),
                offset=10,
            )
            short_dtype = np.dtype(
                [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                 ("scalar_ScanID", "<f4")]
            )
            reservoir_xy = grid_points(
                np.arange(0.025, 0.401, 0.05), [0.025, 0.075]
            )
            reservoir = np.zeros(len(reservoir_xy), short_dtype)
            reservoir["x"], reservoir["y"] = reservoir_xy.T
            reservoir["z"] = 3.0 + np.arange(len(reservoir), dtype=np.float32)
            reservoir["scalar_ScanID"] = 999.0
            terrain = records_for_xy(
                grid_points(np.arange(0.0, 0.41, 0.10), [0.20, 0.30]),
                scan_id=4.0,
                offset=200,
            )
            write_fixture(existing_path, existing)
            write_fixture(reservoir_path, reservoir)
            write_fixture(terrain_path, terrain)
            plan = V11.plan_streaming_water_candidate(
                existing_path,
                reservoir_path,
                [terrain_path],
                review_bbox=[0.0, 0.40, -0.01, 0.15],
                support_margin_m=0.20,
                neighbours=4,
                minimum_spacing_m=0.05,
                maximum_base_spacing_m=0.10,
                maximum_tapered_spacing_m=0.10,
                terrain_support_is_prevalidated=True,
            )
            existing_output = root / "accepted-existing.candidate.ply"
            reservoir_output = root / "accepted-reservoir.candidate.ply"
            result = V11.write_streaming_candidate_parts(
                plan,
                existing_output,
                reservoir_output,
                chunk_size=3,
            )
            existing_result = read_fixture(existing_output)
            reservoir_result = read_fixture(reservoir_output)
            self.assertEqual(existing_result.dtype, existing.dtype)
            self.assertEqual(reservoir_result.dtype, reservoir.dtype)
            self.assertEqual(result.existing_count, len(existing_result))
            self.assertEqual(result.reservoir_count, len(reservoir_result))
            existing_source_bytes = {row.tobytes() for row in existing}
            reservoir_source_bytes = {row.tobytes() for row in reservoir}
            self.assertTrue(
                all(row.tobytes() in existing_source_bytes for row in existing_result)
            )
            self.assertTrue(
                all(row.tobytes() in reservoir_source_bytes for row in reservoir_result)
            )


if __name__ == "__main__":
    unittest.main()
