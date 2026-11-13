# Maintaining openfusion

Read SUMMARY.md first, then this file, README.md, updated/README.md, docs/BUILD.md and relevant implementation/validation evidence.

## Build and validation workflow

Clone recursively or run `git submodule update --init`. The maintained entrypoint is `updated/CMakeLists.txt`, requiring Linux, NVIDIA CUDA and a compatible host compiler, CMake ≥3.18 and C++17. Bundled Eigen and helper headers come from the original submodule. There is no system Eigen, CImg, display-library or sensor-SDK dependency for the headless target. It uses CUDA operators and a CPU Eigen ICP solve.

```sh
cmake -S updated -B build/updated -DCMAKE_BUILD_TYPE=Release
cmake --build build/updated --parallel 2
(cd build/updated && ctest --output-on-failure)
build/updated/openfusion_headless --self-test --demo --output openfusion-output
python3 updated/tools/view_demo.py openfusion-output/demo
```

Default volume is 512³ over 1,024 mm (2 mm voxels). A 128³ build is a coarse 8 mm setting; report its resolution separately. Default CUDA target 86 embeds sm_86 binary code and compute_86 PTX; the checked RTX 4090 ran the build with CUDA 11.5, without recording driver code selection. Select an appropriate target for other hardware/toolkits. Do not change system toolchains or interrupt unrelated GPU jobs just to repeat documentation checks.

For runtime changes, run affected GPU component checks, moving replay and generic replay, retaining commands, environment, outputs and failures. Three CTest entries exercise these paths, not three independent datasets. Review updated/VALIDATION.json, updated/validation/ and implementation before publishing claims. Four Compute Sanitizer tools completed the 128³ component checks and moving demo with no errors/shared-memory hazards; preserve that resolution/path scope. This initcheck version does not check shared-memory initialization. Only record further checks after completed logs. Re-run affected sanitizers after runtime changes or qualify/drop stale claims. Check stationary-pose baseline, per-sphere coverage and visibility errors alongside nearest-surface geometry.

The full-volume CPU/GPU comparison allows only the corresponding weight decision to vary for SDF within ±0.002 mm of the −10 mm integration cutoff, as a separately counted classification band. Preserve outside-band weight/TSDF checks and report differing boundary decisions. This is not a blanket numerical tolerance or general correctness result.

## Preservation and scope

Keep `source/` pinned at `8d435bfff054543c7e57274b773503b1370f90b4`, with original files/notices intact. Adaptations belong under `updated/`. Preserve students Nurit Schwabsky/Vered Cohen and supervisors Chaim Gotsman/Aaron Wetzler attribution alongside upstream owner Gil Ben-Shachar. No overall source license was found; do not add a blanket license to recovered/derived files.

The original entrypoint source/Source/CMakeLists.txt, GUI, camera SDKs and optional filters are unported/unvalidated. Current evidence includes synthetic regressions and all 798 depth frames from one real Kinect TUM Freiburg 1 xyz archive (795 matched mocap frames, three rejected associations). This does not establish other scenes/motions or live-sensor operation. Use docs/REAL_DATA.md and the real-run records before updating that scope. Raycasting retains nearest-voxel sampling, a fixed 1.5 mm step and finite-difference normals with measured plane angular error. `surface.ply` is raycast points, not a mesh. Same-ray range error includes silhouette disagreements and must accompany point-to-scene error in demo accuracy descriptions. Generic replay contains no ground-truth error measurements and records status `executed`, not an accuracy pass; evaluate independent camera ground truth separately when available.

Original-kernel ablations must state whether instrumentation was added: the integration guard fails with the original kernel; original ICP finite-scratch seeding fails its CPU numerical reference and the equivalently seeded updated reduction passes. This diagnostic is not evidence of deterministic unmodified original runtime failure. For TUM imports, use the checked converter's raw/5→millimetre conversion without reapplying dataset depth correction; preserve source timestamps/hashes and note calibration/stride/downsampling. Ground truth is offline only, never supplied to ICP. Anchor at the motion-capture pose interpolated to the first depth timestamp, not the first ground-truth row. Distinguish first-pose-anchored camera error from separately fitted SE(3) position error and from unavailable surface geometry truth. Count matched/rejected timestamp associations and state exact frame range/duration; a completed prefix does not mean the full archive ran. Preserve TUM attribution and CC BY 4.0 dataset terms separately from code terms. Historical imagery is not fresh validation. Label generated input synthetic and distinguish saved-output playback from CUDA execution. Outputs are PPM, ASCII PLY and JSON; view_demo.py embeds these into standalone HTML using Python's standard library.

Before every commit, review and synchronize SUMMARY.md and this file for behavior, workflow, command, output, status and handoff changes. Verify Markdown, relative links, images and diff scope. No packaging step bundles wrapper docs. Request a real Claude editorial review when asked, check factual feedback against code/evidence and record only reviews that actually completed.

The 2026-11-13 real-data follow-up received a completed focused Claude editorial review of its documentation and verified conversion/evaluation facts. Applied corrections clarify timestamp anchoring, capture duration and motion scale; this adds no runtime coverage.
