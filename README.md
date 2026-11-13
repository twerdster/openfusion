# OpenFusion

![Real Kinect recording: RGB reference, measured depth and OpenFusion reconstruction](assets/real-tum-demo.gif)

*Real Kinect recording: RGB reference, measured depth and saved CUDA reconstruction. This excerpt plays saved output; its frame rate does not measure processing speed. TUM Freiburg 1 xyz, Sturm et al., 2012, [CC BY 4.0](https://cvg.cit.tum.de/data/datasets/rgbd-dataset#license). [Recording and evaluation details](docs/REAL_DATA.md).*

![Platform: Linux + NVIDIA CUDA](https://img.shields.io/badge/platform-Linux%20%2B%20NVIDIA%20CUDA-lightgrey)
![Code: C++ / CUDA](https://img.shields.io/badge/code-C%2B%2B%20%2F%20CUDA-blue)
![Original source: preserved](https://img.shields.io/badge/original%20source-preserved-blue)

OpenFusion reconstructs surfaces from calibrated depth recordings using a KinectFusion-style C++/CUDA pipeline. It estimates camera motion with iterative closest point (ICP), combines depth observations in a truncated signed-distance (TSDF) volume, and renders the reconstructed surface. The updated Linux version runs without a display or camera SDK.

## Updated version

Use Linux, an NVIDIA GPU and driver, a compatible CUDA toolkit and host compiler, CMake ≥3.18, and C++17. Initialize the submodule for bundled headers. Python 3 is optional for viewing saved results.

```sh
git clone --recurse-submodules https://github.com/twerdster/openfusion.git
cd openfusion
cmake -S updated -B build/updated -DCMAKE_BUILD_TYPE=Release
cmake --build build/updated --parallel 2
(cd build/updated && ctest --output-on-failure)
build/updated/openfusion_headless --demo --output openfusion-output
python3 updated/tools/view_demo.py openfusion-output/demo
```

Open `openfusion-output/demo/demo.html` to play the saved synthetic frames and rotate the point cloud. Reconstruction runs on CUDA; the browser displays saved output. The default volume is a 1.024 m cube with 2 mm voxels, requiring about 640 MiB of GPU memory plus working buffers.

For your own recordings, use the [replay instructions](docs/BUILD.md#demo-and-file-replay). For real Kinect data, follow the [TUM import and evaluation guide](docs/REAL_DATA.md). The [updated-code README](updated/README.md) describes commands, output files and current limits.

## Results and limits

The Linux build was checked on Ubuntu 22.04, CUDA 11.5 and an RTX 4090. It processed all **798 real Kinect frames** in TUM Freiburg 1 xyz, a desk recording with mostly translational motion, spanning **26.6 seconds of capture**. Camera-position RMSE was **29.5 mm**, versus **239 mm** for a stationary baseline, using 795 motion-capture comparisons; three timestamp associations were excluded. Coordinates were anchored at the first depth timestamp, with ground truth used only for offline evaluation.

Three CTest entries cover component regressions, a synthetic demo and replay of that generated recording. Four Compute Sanitizer tools reported no errors on the 128³ component checks and synthetic demo; 15 Python checks cover import and evaluation tools. [Validation details and recorded evidence](docs/VALIDATION.md) explain the repairs, test scope and measured geometric errors.

The fixed cube covers only part of the room. Real-data surface accuracy remains unmeasured, and no processing-speed benchmark was run. Live sensors, the original GUI, tracking recovery and loop closure are outside the updated version; raycasting retains visible sampling and normal errors.

## Original project and attribution

The recovered code is preserved unchanged in [`source/`](https://github.com/gilbs10/OpenFusion/tree/8d435bfff054543c7e57274b773503b1370f90b4). The [project report](docs/report.pdf) and [archived figure](assets/project-preview.jpg) describe the historical project. Maintenance changes are kept separately in `updated/`.

Students: **Nurit Schwabsky and Vered Cohen**. Supervision: **Chaim Gotsman and Aaron Wetzler**. The pinned upstream repository belongs to [Gil Ben-Shachar (`gilbs10`)](https://github.com/gilbs10).

Original and third-party notices remain in the source and derived files. No overall upstream license file was found; this repository grants no new source license. Authors can open an issue to contribute changes, correct attribution or request removal.

[Project on Aaron Wetzler’s website](https://twerdster.github.io/#openfusion) · [Historical project page](https://gip.cs.technion.ac.il/last-projects-2012-2.html)
