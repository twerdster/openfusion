#!/usr/bin/env python3
"""Small generated fixtures for TUM units, calibration, associations and paths."""

import argparse
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

import numpy as np
from PIL import Image


SPEC = importlib.util.spec_from_file_location(
    "convert_tum", Path(__file__).resolve().parents[1] / "tools" / "convert_tum.py")
CONVERTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONVERTER)


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / "dataset"
        (self.source / "depth").mkdir(parents=True)
        (self.source / "rgb").mkdir()
        self.values = np.full((12, 16), 5000, dtype=np.uint16)
        self.values[0, :8] = [0, 1, 2, 3, 4, 5, 7, 65535]
        for index in range(3):
            Image.fromarray(self.values).save(self.source / f"depth/{index}.png")
            Image.new("RGB", (16, 12), (40 + index, 80, 120)).save(self.source / f"rgb/{index}.png")
        (self.source / "depth.txt").write_text("# fixture\n1.00 depth/0.png\n1.04 depth/1.png\n1.08 depth/2.png\n")
        (self.source / "rgb.txt").write_text("1.01 rgb/0.png\n1.05 rgb/1.png\n1.20 rgb/2.png\n")
        (self.source / "groundtruth.txt").write_text("# preserved, never pose input\n1.0 0 0 0 0 0 0 1\n")

    def tearDown(self):
        self.temporary.cleanup()

    def arguments(self, **changes):
        values = dict(input=self.source, output=self.root / "converted", start=0,
                      frames=3, stride=1, downsample=1, fx=525.0, fy=525.0,
                      cx=319.5, cy=239.5, skip_rgb=False, rgb_max_delta=0.03)
        values.update(changes)
        return argparse.Namespace(**values)

    def test_units_endianness_association_and_ground_truth(self):
        args = self.arguments()
        manifest = CONVERTER.convert(args)
        output = np.fromfile(args.output / "recorded1", dtype="<u2").reshape(12, 16)
        np.testing.assert_array_equal(output[0, :8], [0, 0, 0, 1, 1, 1, 1, 13107])
        self.assertEqual(output[1, 1], 1000)
        self.assertEqual(manifest["intrinsics"]["fx"], -525)
        self.assertEqual(manifest["intrinsics"]["fy"], -525)
        self.assertEqual(manifest["frames"][0]["rgb"]["source_path"], "rgb/0.png")
        self.assertIsNone(manifest["frames"][2]["rgb"])
        self.assertEqual((args.output / "groundtruth.txt").read_bytes(), (self.source / "groundtruth.txt").read_bytes())
        self.assertFalse(manifest["ground_truth"]["used_to_initialize_runtime"])
        self.assertEqual(json.loads((args.output / "manifest.json").read_text()), manifest)

    def test_downsample_is_pixel_stride_with_matching_intrinsics(self):
        args = self.arguments(frames=1, downsample=2)
        manifest = CONVERTER.convert(args)
        expected = ((self.values[::2, ::2].astype(np.uint32) + 2) // 5).astype("<u2")
        actual = np.fromfile(args.output / "recorded1", dtype="<u2").reshape(6, 8)
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(manifest["intrinsics"]["fx"], -262.5)
        self.assertEqual(manifest["intrinsics"]["cx"], 159.75)
        with Image.open(args.output / "rgb/000001.png") as image:
            self.assertEqual(image.size, (8, 6))

    def test_archive_matches_directory_without_extracting(self):
        archive = self.root / "dataset.tgz"
        with tarfile.open(archive, "w:gz") as destination:
            destination.add(self.source, arcname="rgbd_dataset_fixture")
        manifest = CONVERTER.convert(self.arguments(input=archive))
        self.assertEqual(manifest["source"]["kind"], "tgz")
        self.assertEqual(len(manifest["source"]["sha256"]), 64)
        self.assertEqual(manifest["frames"][1]["source_index"], 1)
        self.assertEqual((self.root / "converted/recorded2").stat().st_size, 16 * 12 * 2)

    def test_missing_frames_and_existing_output_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "only 2 are available"):
            CONVERTER.convert(self.arguments(start=1))
        output = self.root / "existing"
        output.mkdir()
        (output / "retain.txt").write_text("keep")
        with self.assertRaisesRegex(ValueError, "already exists"):
            CONVERTER.convert(self.arguments(output=output))
        self.assertEqual((output / "retain.txt").read_text(), "keep")

    def test_non_monotonic_timestamps_and_unsafe_paths_are_rejected(self):
        for text in (b"1 depth/a.png\n1 depth/b.png", b"1 ../a.png", b"nan depth/a.png"):
            with self.assertRaises(ValueError):
                CONVERTER.parse_index(text, "fixture")
        for path in ("/tmp/a.png", "../a.png", "depth/../../a.png", "depth\\a.png"):
            with self.assertRaises(ValueError):
                CONVERTER.safe_relative(path)

    def test_inconsistent_dimensions_and_eight_bit_depth_are_rejected(self):
        Image.new("I;16", (15, 12)).save(self.source / "depth/1.png")
        with self.assertRaisesRegex(ValueError, "inconsistent depth image dimensions"):
            CONVERTER.convert(self.arguments())
        buffer = io.BytesIO()
        Image.new("L", (16, 12)).save(buffer, format="PNG")
        with self.assertRaisesRegex(ValueError, "16-bit PNG"):
            CONVERTER.depth_millimetres(buffer.getvalue())


if __name__ == "__main__":
    unittest.main()
