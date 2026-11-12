# OpenFusion build notes

These notes identify the build entrypoint in pinned upstream commit `8d435bfff054543c7e57274b773503b1370f90b4`. They are not a verified modern installation recipe. The original files in the `source/` submodule are unchanged.

## Requirements and entrypoint

The CMake project is in `source/Source/`. It requires CUDA and an NVIDIA GPU, GLEW, GLUT/OpenGL, X11, POSIX threads, Eigen and CImg. Some headers are bundled upstream; their original terms still apply. The build uses old CUDA SDK utility headers, `FindCUDA` and compute targets 2.0/3.0/3.5. A modern toolchain will need a separately documented port.

An equivalent modern CMake invocation of the original file-replay options is:

```sh
cmake -S source/Source -B build -DBUILD_FROMFILE=ON
cmake --build build
```

The `-S`/`-B` form shown above is modern command-line syntax, not a command recorded in the historical handoff. Configuration was attempted during recovery and failed on the removed CMake 2.6 compatibility policy, before dependencies or numerical code could be validated. Do not interpret these commands as a successful build record.

## Camera backends and recordings

`BUILD_FROMFILE` defaults to `ON`. `BUILD_SOFTKINETIC`, `BUILD_PRIMESENSE` and `BUILD_GIPCAM` default to `OFF`. The CMake cache exposes `SOFTKINETIC_SDK_LIB`, `SOFTKINETIC_SDK_INCLUDE`, `OPENNI_LIB` and `OPENNI_INCLUDE`; sensor SDKs are not supplied by this wrapper. Review `CameraHandler.cpp`, the selected backend and hardcoded paths before attempting capture/replay.

`CameraFromFile.cpp` reads `Parameters.txt` for image dimensions, intrinsic parameters, depth offset and depth limits. Numbered binary depth files are named `recorded1`, `recorded2`, and so on; optional RGB files are `recorded-rgb1`, `recorded-rgb2`, and so on. Compatible recordings are absent. These raw binary formats depend on the source's camera element types and dimensions; arbitrary depth images are not drop-in inputs.

## Recorded scope

No successful modern compilation or runtime has been established. NVIDIA hardware, legacy SDKs and recordings were unavailable. The original report and website figure are historical project records, not fresh runtime/benchmark results. Preserve the pinned source; any compatibility work belongs in a separately labeled updated implementation with its own requirements and validation.
