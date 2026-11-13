# OpenFusion build and recording guide

There are two build entrypoints. `updated/CMakeLists.txt` builds the maintained headless Linux implementation. `source/Source/CMakeLists.txt` belongs to the unchanged historical submodule at `8d435bfff054543c7e57274b773503b1370f90b4`; its old GUI and camera SDK configuration are preserved for reference.

## Updated Linux build

Use a native Linux filesystem for the checkout and build directory. Install a working NVIDIA driver, a CUDA toolkit compatible with that driver and its supported host compiler, CMake 3.18 or newer, and a C++17 compiler. CUDA sources use C++14; host code uses C++17. Initialize the source submodule because the updated build uses its bundled Eigen/CUDA helper headers and an unchanged volume-initialization source file.

```sh
git submodule update --init
cmake -S updated -B build/updated -DCMAKE_BUILD_TYPE=Release
cmake --build build/updated --parallel 2
(cd build/updated && ctest --output-on-failure)
build/updated/openfusion_headless --self-test --demo --output openfusion-output
python3 updated/tools/view_demo.py openfusion-output/demo
```

The checked environment is Ubuntu 22.04, GCC 11.4, CMake 3.22.1, CUDA 11.5.119, driver 570.211.01 and an RTX 4090. The declared CMake minimum is 3.18; only 3.22.1 was executed. The initial recorded build used `RelWithDebInfo`; an isolated clean `Release` build also passed all three CTest entries. These are recorded versions, not a claim that every toolchain combination works. If CMake cannot find the toolkit, set `CUDACXX` to the intended `nvcc` executable before the first configuration, or use a fresh build directory after changing toolchains. A CPU-only machine cannot execute this implementation.

The default CUDA architecture is 86. It embeds sm_86 binary code and compute_86 PTX, and the build ran on the test machine's Ada GPU. The run did not record which code form the driver selected. On another GPU/toolkit combination, select a target supported by both, for example:

```sh
cmake -S updated -B build/native \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
```

The example target 89 requires a toolkit that supports it. It is not the command used with CUDA 11.5. No system Eigen, GLEW, GLUT, OpenGL, X11 or CImg dependency is needed for the updated headless target.

## Volume configuration and resources

The default `OPENFUSION_VOLUME_SIZE=512` preserves the original dimension: a 1,024 mm cube with 2 mm voxels. Float TSDF values and byte weights use about 640 MiB on the GPU, before camera buffers, ICP allocations, guard slabs and host copies. Full-volume reference comparisons also require host memory.

For a coarse regression run:

```sh
cmake -S updated -B build/coarse \
  -DCMAKE_BUILD_TYPE=Release -DOPENFUSION_VOLUME_SIZE=128
cmake --build build/coarse --parallel 2
(cd build/coarse && ctest -R '^(gpu_regression|gpu_demo)$' --output-on-failure)
```

Only 512³ and 128³ were executed. At 128³, self-test and demo were checked; generic file replay was checked only at 512³. Other accepted dimensions (16–1024) are unvalidated; 1024³ needs about 5 GiB for TSDF/weights alone. The 128³ volume has 8 mm voxels across the same cube. It changes sampling, tracking behavior and appearance; it is not equivalent quality at lower memory use. Its banding is visible. Keep results from different volume sizes separate.

## Demo and file replay

`--self-test` runs component checks and `--demo` generates and processes the moving synthetic recording. With no `--self-test`, `--demo` or `--recording` option, self-test and demo both run. CTest runs three entries: component regressions, the demo, and generic file replay of the generated recording. The replay test depends on the demo's output.

`--demo` writes a `demo/` directory below `--output`. Its `recording/` contains generated original-format depth frames and calibration. `input-depth.ppm` and `input-normals.ppm` show the final input frame; `reconstruction-NN.ppm` are per-frame CUDA raycasts, and `reconstruction.ppm` is the final one. `surface.ply` is an ASCII point cloud of final raycast hits with normals, in millimetres; it is not a triangulated mesh. `metrics.json` contains per-frame estimated and ground-truth poses, errors, coverage and integration diagnostics.

To process a compatible recording explicitly:

```sh
build/updated/openfusion_headless \
  --recording openfusion-output/demo/recording --frames 12 \
  --output replay-output
python3 updated/tools/view_demo.py replay-output/replay \
  --input-label "replay of the generated synthetic recording"
```

Generic replay has no independent ground truth and records `ground_truth_available: false`, with status `executed`. The replay test requires a finite estimated pose and normals, nonempty raycast surface, successful subsequent ICP solves and unchanged allocation guards; producing output is not an accuracy test. It uses a fixed 1,024 mm cube centered at the world origin and an initial camera position of (0, 0, 768) mm; it does not infer a scene origin from the recording. It stops before integrating a frame whose ICP solve is rejected. It writes below `replay/`; inspect the JSON and tracking status before drawing conclusions.

The viewer uses Python's standard library to embed PPM, ASCII PLY and JSON output into `demo.html`, with saved-frame playback and a rotatable point cloud. Its browser view requires neither a GPU nor a server. It displays previously computed results and does not execute reconstruction interactively.

## Recording format

The checked reader accepts a directory containing `Parameters.txt` and `recorded1`, `recorded2`, and so on. Required whitespace-separated settings are:

```text
width height
fx fy cx cy
z_offset
minimum_depth_mm
maximum_depth_mm
```

The historical file can contain additional trailing information. The updated reader checks dimensions, finite calibration, nonzero focal lengths, usable depth limits and exact frame byte counts. `z_offset` must be zero: the reconstruction path does not apply it, so accepting another value would imply a correction that never happens. The original convention looks along negative camera z; the generated fixture uses negative focal lengths consistent with its image axes. Changing these signs without converting conventions changes the geometry.

Depth files are raw arrays of `uint16` axial depths in millimetres, `width × height` elements in row-major order. Optional `recorded-rgbN` files are raw `uint32` elements with the same dimensions. RGB channel packing depends on the original backend and is not used for fusion in this runner. This format has no endianness tag and is read in host byte order; only little-endian execution was checked. The reader rejects truncated and oversized payloads and reports end of input rather than treating it as a valid frame. A PNG/JPEG or a dataset's native calibration file must be converted explicitly before replay.

## Historical application

The original project requires CUDA, GLEW, GLUT/OpenGL, X11, POSIX threads, Eigen and CImg, plus the selected camera SDK. Bundled headers retain their individual terms. It uses old CUDA utility headers, `FindCUDA`, and compute targets 2.0/3.0/3.5. The [Linux configuration attempt](../updated/validation/original-configure.log) with CMake 3.22.1 stopped at missing GLEW. Separately, a macOS configuration check with CMake 4.0.3 failed because compatibility below CMake 3.5 has been removed, while the original project declares 2.6. Neither failure is a result from the updated entrypoint. The original application remains unvalidated with the modern toolchain.

```sh
cmake -S source/Source -B build/original -DBUILD_FROMFILE=ON
```

This identifies the original replay option; it is not the maintained build recipe. `BUILD_FROMFILE` defaults to `ON`; `BUILD_SOFTKINETIC`, `BUILD_PRIMESENSE` and `BUILD_GIPCAM` default to `OFF`. Historical cache options include `SOFTKINETIC_SDK_LIB`, `SOFTKINETIC_SDK_INCLUDE`, `OPENNI_LIB` and `OPENNI_INCLUDE`. Review original `CameraHandler.cpp`, selected backends and hardcoded capture paths before attempting that application. No live sensor or GUI coverage follows from the headless checks.
