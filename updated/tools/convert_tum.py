#!/usr/bin/env python3
"""Convert selected TUM RGB-D depth PNGs to OpenFusion's recording format.

Requires NumPy and Pillow. Input may be an extracted dataset directory or its
official .tgz archive. No files are extracted with tarfile.extract/extractall.
TUM stores 5,000 units per metre; its published PNGs already include the dataset's
depth correction. This converter applies only the documented units conversion.
Ground truth is copied for later evaluation and never initializes the runtime.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import tarfile

import numpy as np
from PIL import Image


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def safe_relative(name: str) -> str:
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts or "\\" in name:
        raise ValueError(f"unsafe dataset path: {name!r}")
    return str(path)


class Dataset:
    """Read a directory or selected regular members of a gzip tar archive."""

    def __init__(self, path: Path):
        self.path = path.resolve()
        self.archive = not self.path.is_dir()
        self.cache: dict[str, bytes] = {}
        self.prefix = ""
        if self.archive:
            candidates = []
            text_members = {}
            with tarfile.open(self.path, "r|gz") as archive:
                for member in archive:
                    name = safe_relative(member.name)
                    if not member.isfile():
                        continue
                    if PurePosixPath(name).name in {"depth.txt", "rgb.txt", "groundtruth.txt"}:
                        if member.size > 16 * 1024 * 1024:
                            raise ValueError(f"dataset metadata is unexpectedly large: {name}")
                        text_members[name] = archive.extractfile(member).read()
                        if PurePosixPath(name).name == "depth.txt":
                            candidates.append(name)
            if len(candidates) != 1:
                raise ValueError("archive must contain exactly one depth.txt")
            parent = str(PurePosixPath(candidates[0]).parent)
            self.prefix = "" if parent == "." else parent + "/"
            for name, data in text_members.items():
                if name.startswith(self.prefix):
                    self.cache[name[len(self.prefix):]] = data

    def read(self, name: str) -> bytes:
        name = safe_relative(name)
        if not self.archive:
            candidate = (self.path / name).resolve()
            if not candidate.is_relative_to(self.path):
                raise ValueError(f"dataset path escapes directory: {name}")
            return candidate.read_bytes()
        if name not in self.cache:
            raise FileNotFoundError(f"archive member not loaded: {name}")
        return self.cache[name]

    def optional(self, name: str) -> bytes | None:
        try:
            return self.read(name)
        except FileNotFoundError:
            return None

    def prepare(self, names: set[str]) -> None:
        if not self.archive:
            return
        wanted = {self.prefix + safe_relative(name): name for name in names}
        with tarfile.open(self.path, "r|gz") as archive:
            for member in archive:
                name = safe_relative(member.name)
                if name in wanted:
                    if not member.isfile() or member.size > 32 * 1024 * 1024:
                        raise ValueError(f"expected a regular image member: {name}")
                    self.cache[wanted.pop(name)] = archive.extractfile(member).read()
        if wanted:
            raise FileNotFoundError(f"archive is missing selected members: {list(wanted)[:3]}")


def parse_index(data: bytes, label: str) -> list[tuple[float, str, str]]:
    rows = []
    for line_number, line in enumerate(data.decode("utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 2:
            raise ValueError(f"{label}:{line_number}: expected timestamp and relative PNG path")
        timestamp = float(fields[0])
        if not math.isfinite(timestamp):
            raise ValueError(f"{label}:{line_number}: timestamp must be finite")
        if rows and timestamp <= rows[-1][0]:
            raise ValueError(f"{label}:{line_number}: timestamps must increase strictly")
        rows.append((timestamp, fields[0], safe_relative(fields[1])))
    if not rows:
        raise ValueError(f"{label} contains no frames")
    return rows


def nearest_rgb(rows, timestamp: float, tolerance: float):
    if not rows:
        return None
    times = [row[0] for row in rows]
    position = bisect.bisect_left(times, timestamp)
    candidates = rows[max(0, position - 1):min(len(rows), position + 1)]
    nearest = min(candidates, key=lambda row: abs(row[0] - timestamp))
    return nearest if abs(nearest[0] - timestamp) <= tolerance else None


def depth_millimetres(png: bytes, downsample: int = 1) -> np.ndarray:
    with Image.open(io.BytesIO(png)) as image:
        if image.format != "PNG" or image.mode not in {"I", "I;16", "I;16B", "I;16L"}:
            raise ValueError("depth must be a single-channel 16-bit PNG")
        values = np.asarray(image)
        if values.ndim != 2 or values.size == 0 or values.min() < 0 or values.max() > 65535:
            raise ValueError("depth PNG values must lie in the unsigned 16-bit range")
        # Integer nearest rounding: /5 never produces an exact half-integer.
        # Zero remains invalid; no extra scale factor, clipping or filling.
        return ((values[::downsample, ::downsample].astype(np.uint32) + 2) // 5).astype("<u2")


def convert(args: argparse.Namespace) -> dict:
    if args.output.exists():
        raise ValueError("output path already exists; choose a new directory")
    if args.start < 0 or args.frames <= 0 or args.stride <= 0 or args.downsample <= 0:
        raise ValueError("start must be nonnegative; frames, stride and downsample must be positive")
    if not all(math.isfinite(v) for v in (args.fx, args.fy, args.cx, args.cy, args.rgb_max_delta)):
        raise ValueError("calibration and RGB tolerance must be finite")
    if args.fx <= 0 or args.fy <= 0 or args.rgb_max_delta < 0:
        raise ValueError("TUM focal lengths must be positive and RGB tolerance nonnegative")
    dataset = Dataset(args.input)
    all_depth = parse_index(dataset.read("depth.txt"), "depth.txt")
    selected = list(enumerate(all_depth))[args.start:args.start + args.frames * args.stride:args.stride]
    if not selected or len(selected) != args.frames:
        raise ValueError(f"requested {args.frames} frames, but only {len(selected)} are available")
    rgb_index = dataset.optional("rgb.txt")
    rgb_rows = parse_index(rgb_index, "rgb.txt") if rgb_index is not None and not args.skip_rgb else []
    associations = [nearest_rgb(rgb_rows, row[0], args.rgb_max_delta) for _, row in selected]
    wanted = {row[2] for _, row in selected}
    wanted.update(row[2] for row in associations if row is not None)
    dataset.prepare(wanted)
    first_png = dataset.read(selected[0][1][2])
    with Image.open(io.BytesIO(first_png)) as image:
        input_width, input_height = image.size
    first_depth = depth_millimetres(first_png, args.downsample)
    height, width = first_depth.shape
    if width <= 4 or height <= 4 or width > 4096 or height > 4096:
        raise ValueError("converted dimensions must be 5–4096 pixels in each direction")
    output = args.output.resolve()
    output.mkdir(parents=True)
    if any(associations):
        (output / "rgb").mkdir()
    focal_x = -args.fx / args.downsample
    focal_y = -args.fy / args.downsample
    center_x = args.cx / args.downsample
    center_y = args.cy / args.downsample
    calibration = (f"{width} {height}\n{focal_x:.12g} {focal_y:.12g} "
                   f"{center_x:.12g} {center_y:.12g}\n0\n1\n65535\n{len(selected)}\n"
                   "TUM RGB-D imported depth; source timestamps in manifest.json\n")
    (output / "Parameters.txt").write_text(calibration, encoding="utf-8")
    frames = []
    for frame_number, ((source_index, row), rgb_row) in enumerate(zip(selected, associations), 1):
        png = dataset.read(row[2])
        with Image.open(io.BytesIO(png)) as image:
            if image.size != (input_width, input_height):
                raise ValueError(f"inconsistent depth image dimensions: {row[2]}")
        depth = first_depth if frame_number == 1 else depth_millimetres(png, args.downsample)
        data = depth.tobytes(order="C")
        recorded = f"recorded{frame_number}"
        (output / recorded).write_bytes(data)
        frame = {"frame": frame_number, "source_index": source_index,
                 "depth_timestamp": row[0], "depth_timestamp_text": row[1],
                 "source_depth_path": row[2], "source_png_sha256": sha256(png),
                 "recorded_file": recorded, "recorded_sha256": sha256(data),
                 "valid_depth_pixels": int(np.count_nonzero(depth)), "rgb": None}
        if rgb_row is not None:
            rgb_png = dataset.read(rgb_row[2])
            rgb_file = f"rgb/{frame_number:06d}.png"
            if args.downsample == 1:
                with Image.open(io.BytesIO(rgb_png)) as image:
                    if image.size != (input_width, input_height) or image.format != "PNG":
                        raise ValueError(f"RGB dimensions/format differ from depth: {rgb_row[2]}")
                (output / rgb_file).write_bytes(rgb_png)
            else:
                with Image.open(io.BytesIO(rgb_png)) as image:
                    if image.size != (input_width, input_height):
                        raise ValueError(f"RGB dimensions differ from depth: {rgb_row[2]}")
                    image = Image.fromarray(np.asarray(image.convert("RGB"))[::args.downsample, ::args.downsample])
                    image.save(output / rgb_file)
            frame["rgb"] = {"timestamp": rgb_row[0], "source_path": rgb_row[2],
                            "delta_seconds": rgb_row[0] - row[0],
                            "source_png_sha256": sha256(rgb_png), "output_file": rgb_file}
        frames.append(frame)
    truth = dataset.optional("groundtruth.txt")
    ground_truth = None
    if truth is not None:
        (output / "groundtruth.txt").write_bytes(truth)
        ground_truth = {"file": "groundtruth.txt", "sha256": sha256(truth),
                        "used_to_initialize_runtime": False}
    (output / "source-depth.txt").write_bytes(dataset.read("depth.txt"))
    if rgb_index is not None:
        (output / "source-rgb.txt").write_bytes(rgb_index)
    source = {"path": str(dataset.path), "kind": "tgz" if dataset.archive else "directory"}
    if dataset.archive:
        digest = hashlib.sha256()
        with dataset.path.open("rb") as archive_file:
            for block in iter(lambda: archive_file.read(4 * 1024 * 1024), b""):
                digest.update(block)
        source["sha256"] = digest.hexdigest()
    manifest = {"schema_version": 1, "dataset": "TUM RGB-D", "source": source,
                "format_documentation": "https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats",
                "depth_conversion": {"source_units_per_metre": 5000, "output_units": "millimetres",
                                     "operation": "round(raw_uint16 / 5), little-endian uint16",
                                     "invalid_source_value": 0, "additional_depth_scale_applied": False,
                                     "valid_depth_mask_applied": False},
                "selection": {"start": args.start, "frames": args.frames, "stride": args.stride,
                              "downsample": args.downsample, "sampling": "pixel stride, no interpolation",
                              "rgb_max_delta_seconds": args.rgb_max_delta},
                "image": {"width": width, "height": height,
                          "source_width": input_width, "source_height": input_height},
                "intrinsics": {"fx": focal_x, "fy": focal_y, "cx": center_x, "cy": center_y,
                               "source_tum": {"fx": args.fx, "fy": args.fy, "cx": args.cx, "cy": args.cy},
                               "model": "registered pinhole, no distortion correction applied"},
                "camera_coordinate_transform": {"tum_to_openfusion": [[1, 0, 0], [0, 1, 0], [0, 0, -1]],
                                                "metres_to_mm": 1000,
                                                "description": "TUM +z forward -> OpenFusion -z forward; x right, y down"},
                "runtime_pose": {"ground_truth_used": False,
                                 "initial_pose": "chosen by openfusion_headless, not this converter"},
                "ground_truth": ground_truth, "frames": frames}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="extracted TUM dataset directory or official .tgz archive")
    parser.add_argument("output", type=Path, help="new output directory (existing paths are rejected)")
    parser.add_argument("--start", type=int, default=0, help="zero-based source depth frame index")
    parser.add_argument("--frames", type=int, default=150)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--downsample", type=int, default=1, help="integer pixel stride, no interpolation")
    parser.add_argument("--fx", type=float, default=525.0, help="positive TUM pinhole focal length in pixels")
    parser.add_argument("--fy", type=float, default=525.0, help="positive TUM pinhole focal length in pixels")
    parser.add_argument("--cx", type=float, default=319.5)
    parser.add_argument("--cy", type=float, default=239.5)
    parser.add_argument("--skip-rgb", action="store_true", help="omit optional associated RGB preview PNGs")
    parser.add_argument("--rgb-max-delta", type=float, default=0.03, help="maximum RGB/depth timestamp separation in seconds")
    args = parser.parse_args()
    try:
        manifest = convert(args)
    except (ValueError, OSError, tarfile.TarError) as error:
        parser.exit(1, f"Conversion failed: {error}\n")
    print(json.dumps({"output": str(args.output.resolve()), "frames": len(manifest["frames"]),
                      "width": manifest["image"]["width"], "height": manifest["image"]["height"]}))


if __name__ == "__main__":
    main()
