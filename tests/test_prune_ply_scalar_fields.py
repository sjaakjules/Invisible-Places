import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "prune_ply_scalar_fields.py"
SPEC = importlib.util.spec_from_file_location("prune_ply_scalar_fields", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class PrunePlyScalarFieldsTests(unittest.TestCase):
    def test_streaming_rewrite_preserves_retained_values(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.ply"
            output = root / "cleaned.ply"
            dtype = np.dtype(
                [
                    ("x", "<f4"),
                    ("red", "u1"),
                    ("scalar_Keep", "<f4"),
                    ("scalar_Remove", "<f4"),
                ]
            )
            points = np.zeros(10, dtype=dtype)
            points["x"] = np.arange(10, dtype=np.float32)
            points["red"] = np.arange(10, dtype=np.uint8)
            points["scalar_Keep"] = np.arange(10, dtype=np.float32) * 2.0
            points["scalar_Remove"] = -np.arange(10, dtype=np.float32)
            with source.open("wb") as stream:
                stream.write(
                    b"ply\n"
                    b"format binary_little_endian 1.0\n"
                    b"element vertex 10\n"
                    b"property float x\n"
                    b"property uchar red\n"
                    b"property float scalar_Keep\n"
                    b"property float scalar_Remove\n"
                    b"end_header\n"
                )
                points.tofile(stream)

            report = MODULE.rewrite_ply(
                source,
                output,
                {"scalar_Remove", "scalar_Missing"},
                64,
            )
            cleaned = MODULE.inspect_ply(output)
            self.assertEqual(
                [prop.name for prop in cleaned.properties],
                ["x", "red", "scalar_Keep"],
            )
            self.assertEqual(report["removed_properties"], ["scalar_Remove"])
            self.assertEqual(
                report["requested_but_absent"], ["scalar_Missing"]
            )
            values = np.memmap(
                output,
                mode="r",
                dtype=cleaned.dtype,
                offset=len(cleaned.header_bytes),
                shape=(10,),
            )
            np.testing.assert_array_equal(values["x"], points["x"])
            np.testing.assert_array_equal(
                values["scalar_Keep"], points["scalar_Keep"]
            )

    def test_downhill_vector_can_be_reduced_to_slope_and_azimuth(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.ply"
            output = root / "cleaned.ply"
            dtype = np.dtype(
                [
                    ("x", "<f4"),
                    ("scalar_A_R_Downhill_X", "<f4"),
                    ("scalar_A_R_Downhill_Y", "<f4"),
                    ("scalar_A_R_Downhill_Z", "<f4"),
                    ("scalar_A_R_DownhillMagnitude", "<f4"),
                    ("scalar_A_R_Horizontalness", "<f4"),
                    ("scalar_A_R_Slope_deg", "<f4"),
                ]
            )
            points = np.zeros(3, dtype=dtype)
            points["scalar_A_R_Downhill_X"] = [1.0, 0.0, -1.0]
            points["scalar_A_R_Downhill_Y"] = [0.0, 1.0, 0.0]
            points["scalar_A_R_Downhill_Z"] = -0.5
            points["scalar_A_R_DownhillMagnitude"] = 0.5
            points["scalar_A_R_Horizontalness"] = 0.8660254
            points["scalar_A_R_Slope_deg"] = 30.0
            with source.open("wb") as stream:
                stream.write(
                    b"ply\nformat binary_little_endian 1.0\n"
                    b"element vertex 3\nproperty float x\n"
                    b"property float scalar_A_R_Downhill_X\n"
                    b"property float scalar_A_R_Downhill_Y\n"
                    b"property float scalar_A_R_Downhill_Z\n"
                    b"property float scalar_A_R_DownhillMagnitude\n"
                    b"property float scalar_A_R_Horizontalness\n"
                    b"property float scalar_A_R_Slope_deg\nend_header\n"
                )
                points.tofile(stream)

            removed = {
                "scalar_A_R_Downhill_X",
                "scalar_A_R_Downhill_Y",
                "scalar_A_R_Downhill_Z",
                "scalar_A_R_DownhillMagnitude",
                "scalar_A_R_Horizontalness",
            }
            MODULE.rewrite_ply(source, output, removed, 64, True)
            cleaned = MODULE.inspect_ply(output)
            self.assertEqual(
                [prop.name for prop in cleaned.properties],
                [
                    "x",
                    "scalar_A_R_Slope_deg",
                    "scalar_A_R_Downhill_Azimuth_deg",
                ],
            )
            values = np.memmap(
                output,
                mode="r",
                dtype=cleaned.dtype,
                offset=len(cleaned.header_bytes),
                shape=(3,),
            )
            np.testing.assert_allclose(
                values["scalar_A_R_Downhill_Azimuth_deg"],
                [0.0, 90.0, 180.0],
                atol=1.0e-5,
            )
            np.testing.assert_array_equal(
                values["scalar_A_R_Slope_deg"],
                points["scalar_A_R_Slope_deg"],
            )


if __name__ == "__main__":
    unittest.main()
