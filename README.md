# OpenFusion

<p align="center">
  <img src="assets/project-preview.jpg" alt="Four-panel figure: Phong-shaded 3D reconstruction of a seated person, raw normal map from input, depth image from input, and the camera RGB image" width="480">
</p>

![Platform: Linux + NVIDIA CUDA](https://img.shields.io/badge/platform-Linux%20%2B%20NVIDIA%20CUDA-lightgrey)
![Code: C++ / CUDA](https://img.shields.io/badge/code-C%2B%2B%20%2F%20CUDA-blue)
![Source: pinned upstream submodule](https://img.shields.io/badge/source-pinned%20upstream%20submodule-blue)

OpenFusion implements a KinectFusion-style depth reconstruction pipeline in C++/CUDA. Depth frames are converted to vertex/normal maps, aligned with iterative closest point (ICP), integrated into a truncated signed-distance volume (TSDF), and raycast for display. A camera interface separates file replay from sensor backends; it does not make arbitrary cameras compatible without an implementation and calibration.

Students: Nurit Schwabsky, Vered Cohen. Supervision: Aaron Wetzler and Chaim Gotsman. Pinned upstream repository: [Gil Ben-Shachar (`gilbs10`)](https://github.com/gilbs10).

*The four-panel historical figure shows a reconstruction, normal map, depth image and camera RGB image. It is unchanged archived imagery, not a result produced during maintenance.*

## Original code

The [35-page project report](docs/report.pdf) is preserved alongside [gilbs10/OpenFusion](https://github.com/gilbs10/OpenFusion), pinned as the `source/` Git submodule at `8d435bfff054543c7e57274b773503b1370f90b4`. The upstream files, bundled headers and notices are unchanged.

`CameraHandler`, `ICP`, `VolumeIntegrator` and `Raycaster` implement the input/alignment/integration/display chain under `source/Source/`. File replay expects a `Parameters.txt` calibration/settings file and numbered binary `recordedN` depth files; optional RGB files use `recorded-rgbN`. These are the program's recording formats, not an included dataset. Sensor backends include OpenNI/PrimeSense and SoftKinetic/DepthSense.

## Inspect and build

```sh
git clone --recurse-submodules https://github.com/twerdster/openfusion.git
cd openfusion
```

The historical Linux build requires an NVIDIA CUDA GPU/toolkit, Eigen, CImg, GLEW, GLUT, OpenGL, X11 and POSIX threads. Camera SDKs or compatible recordings must be supplied separately. The CMake entrypoint is `source/Source/CMakeLists.txt`; see [build notes](docs/BUILD.md) for modern invocations of the original options, backend switches and known blockers.

There is no verified modern installation recipe. A historical 2014 handoff described Visual Studio 2010/CUDA 5.5. No Visual Studio solution is present in the pinned revision.

## Validation and limitations

The recorded local CMake configuration attempt failed before compilation because current CMake removed compatibility with the declared 2.6 minimum. The source also uses legacy CUDA utility headers and targets compute capabilities 2.0, 3.0 and 3.5. Recordings, compatible SDKs and NVIDIA hardware were unavailable for runtime validation.

No successful modern compilation, numerical comparison, sensor run, reconstruction benchmark or performance measurement was recorded. Hardcoded capture paths and sensor settings need review. This archive contains no updated implementation or notebook; the original figure and report are historical evidence only.

## Attribution and terms

Student and third-party notices remain in the pinned source. No overall upstream license file was found; individual headers may carry their own terms, and this wrapper grants no new source license. Authors can open an issue to contribute changes, correct attribution or request removal.

## Links

- [Project entry on Aaron Wetzler's website](https://twerdster.github.io/#openfusion)
- [Original source](https://github.com/gilbs10/OpenFusion)
- [Recovered project report](docs/report.pdf)
- [Project page](https://gip.cs.technion.ac.il/last-projects-2012-2.html)
