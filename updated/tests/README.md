# GPU regression checks

The `openfusion_headless` executable links the adapted OpenFusion CUDA
operators. Its CPU code generates analytic input and computes independent
references; it does not perform reconstruction in place of the GPU operators.
An NVIDIA GPU and working CUDA runtime are required. These checks do not have
a CPU fallback.

Run the registered regression, moving demo and file-replay checks after building:

```sh
(cd build/updated && ctest --output-on-failure)
```

Or run the checks and moving replay together, keeping their output:

```sh
./build/updated/openfusion_headless --self-test --demo --output output
```

## What the checks cover

- **Recording and depth conversion:** write the original `Parameters.txt` and
  `recorded1` format, read it through `CameraFromFile`, and compare every raw
  unsigned 16-bit depth and unsigned 32-bit RGB value. Compare GPU vertices and
  normals against CPU unprojection and cross products at 97 by 73 pixels.
  Include invalid depth values, image boundaries, missing frames, truncated or
  oversized depth files, incomplete calibration, invalid focal lengths and
  dimensions, reversed depth limits and unsupported nonzero depth offsets.
- **ICP correspondences:** compare each GPU block's symmetric 6 by 6 normal
  matrix and six-element right-hand side against independently accumulated CPU
  values. The fixture contains invalid pixels and a width and height that leave
  partial 16 by 16 blocks. This exercises reduction values and synchronization
  at image boundaries, rather than accepting an unchanged pose as proof of ICP.
- **TSDF integration:** compare every voxel's signed distance and weight against
  the original radial-distance integration formula. A wide field of view reaches
  the final x slab. Guard slabs before and after both GPU arrays detect writes
  outside the volume, including the recovered code's one-slab indexing error.
  Weight decisions remain exact except when the CPU reference SDF is within
  0.002 mm of the hard negative truncation boundary. In that narrow band, either
  the unchanged state or an updated negative-boundary value is accepted; the
  executable reports how many voxels occupy the band and how many CPU/GPU
  threshold decisions differ. TSDF value comparisons keep the same tolerance.
- **Raycasting:** reconstruct a plane with known world-space position. Check ray
  coverage and position error. Compare GPU normals against an independent CPU
  central difference of the downloaded TSDF; require finite unit or zero vectors.
  Report the angle to the analytic plane normal separately. This checks the
  implemented normal operator, not analytic normal accuracy: radial TSDF values
  use nearest-pixel samples, which can create tilted, reversed or zero gradients.
- **Motion estimation and rejection:** estimate a known nonzero translation and
  rotation in an asymmetric three-sphere scene. Require an actual solver success
  status and improvement over the initial pose. A separate plane fixture has
  unconstrained motions; ICP must reject it and retain its input pose.
- **Camera-facing integration:** place a camera inside the volume, integrate a
  plane in front of it, and verify every behind-camera voxel remains unobserved.

The tolerances for depth conversion, normal-equation accumulation and TSDF
integration cover floating-point evaluation differences. Raycast tolerances
include voxel width and the inherited 1.5 mm marching step. The executable logs
the observed values. One CTest test invokes this set of checks; its count is not
a count of independent datasets or camera backends.

## Moving replay and error measurements

The demo analytically raytraces three asymmetric spheres and a back plane,
quantizes camera-axis depth to integer millimeters, and writes 12 frames in the
original recording format. The GPU chain reads those frames, converts depth to
vertices and normals, estimates pose with ICP, integrates the TSDF and raycasts
the result. Ground-truth poses are used for evaluation, not to update estimated
poses after initialization.

`demo/metrics.json` records each estimated and ground-truth pose, solver status,
ray coverage, observed voxel counts and integration weights. Final estimated
translation must improve on a stationary-camera baseline by at least 25 percent,
and estimated travel must exceed half the true displacement. This prevents a
finite but stationary pose from passing the moving demo.

Two geometry measurements are recorded separately:

- **Point-to-surface distance** is the minimum Euclidean distance from each GPU
  raycast point to the known analytic sphere or back-plane surfaces. RMSE, 95th
  percentile and maximum are in millimeters. Each frame must satisfy the stated
  voxel-scale RMSE and percentile limits.
- **Per-ray distance** compares the GPU hit with the ideal first intersection at
  the same pixel and estimated camera pose. It measures visibility as well as
  geometry. A coarse voxel can move a silhouette across a pixel, changing its
  first hit between a foreground sphere and the distant plane. These large
  errors remain visible in the RMSE, percentile and outlier fraction. Fewer than
  5 percent of hit pixels may exceed two voxel widths plus the 1.5 mm marching
  step. This metric is not replaced by point-to-surface distance.

Every frame must also contain at least 100 raycast samples near each of the
three sphere surfaces. The surface counts are saved in the JSON. Reconstructing
only the background plane cannot satisfy that check, even if its minimum
point-to-surface distance is small.

The demo saves input depth, input normals and GPU reconstruction as PPM files,
per-frame reconstructions, and `surface.ply`. The ASCII PLY contains final
raycast samples and normals in millimeters; it is a point cloud, not a mesh.
The raw fixture and its calibration are kept under `demo/recording/`.

## Supplied recordings

The same executable can process recordings other than the generated fixture:

```sh
./build/updated/openfusion_headless --recording /path/to/recording --frames 12 --output output
```

It reads the specified number of `recordedN` frames through `CameraFromFile`,
tracks, integrates and raycasts each one, and writes images, a point cloud and
`replay/metrics.json`. A missing or malformed frame, rejected ICP solve, nonfinite
output or empty reconstruction returns failure. A rejected pose is not used to
integrate another frame. The initial camera pose remains the historical
translation of 768 mm along world +z; the fixed reconstruction cube is 1024 mm
wide. Recordings must be calibrated in the original negative-camera-z convention
and in millimeters, with a zero depth offset. This interface does not import
arbitrary PNG depth images or apply lens-distortion correction.

Without independent ground truth, that mode reports operator execution and
estimated poses only. Its JSON explicitly sets `ground_truth_available` to false
and marks a completed run `executed`; it does not fabricate accuracy metrics.

## Limits

These are small, deterministic synthetic fixtures. They exercise the recovered
algorithm chain and specific defects, but do not establish reconstruction
quality on real sensor recordings, tracking under large motions or occlusions,
camera SDK compatibility, historical GUI operation or benchmark performance.
The 128-cubed test volume has 8 mm voxels in a 1024 mm cube; the original
512-cubed volume has 2 mm voxels. Coarse finite differences can produce visible
shading bands, and nearest-voxel raycasting retains silhouette errors. Changing
the volume size changes those limits and is recorded in every demo metric file.
