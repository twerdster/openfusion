# openfusion: maintenance context

Read this file first, then SKILL.md and README.md. Context updated 2026-11-13.

## Purpose and provenance

OpenFusion implements calibrated depth conversion, ICP alignment, TSDF integration and raycasting in C++/CUDA. `source/` remains the unchanged gilbs10/OpenFusion submodule at `8d435bfff054543c7e57274b773503b1370f90b4`. The recovered 35-page report credits students Nurit Schwabsky/Vered Cohen and supervisors Chaim Gotsman/Aaron Wetzler; the upstream repository owner is Gil Ben-Shachar (`gilbs10`). Preserve both records without treating the names as interchangeable. Individual source/third-party terms remain; no overall source license was found.

## Updated implementation and commands

`updated/` is the separately labeled 2026 Linux headless port. CUDA executes depth conversion/normals, ICP correspondence reduction, TSDF integration and raycasting; the ICP 6 × 6 solve uses CPU Eigen. Bundled headers and unchanged original General.cu are required, so initialize the submodule. No system Eigen/CImg/GLEW/GLUT/X11 or camera SDK is needed for this target.

From the repository root:

```sh
git submodule update --init
cmake -S updated -B build/updated -DCMAKE_BUILD_TYPE=Release
cmake --build build/updated --parallel 2
(cd build/updated && ctest --output-on-failure)
build/updated/openfusion_headless --self-test --demo --output openfusion-output
python3 updated/tools/view_demo.py openfusion-output/demo
```

Default volume is 512³ across 1,024 mm (2 mm voxels); 128³ uses 8 mm voxels and is a coarse setting with substantial banding. CUDA target defaults to 86, embedding sm_86 binary code and compute_86 PTX (driver selection unrecorded); choose another target/toolkit together. Generic replay uses `--recording DIR --frames N --output OUT` and writes `OUT/replay`, without ground-truth accuracy claims. See docs/BUILD.md and updated/README.md.

Repairs cover TSDF x-plane indexing/guard overrun and behind-camera rejection, bounded/symmetric ICP reduction and partial-block barriers, evaluated inverses, finite/SPD solve status, array cleanup, validated replay and raycast miss clearing. Original source pin/bytes are unchanged.

## Validation state

Compiled and executed on Ubuntu 22.04, GCC 11.4, CMake 3.22.1, CUDA 11.5.119, driver 570.211.01, RTX 4090. Three CTest entries cover component regressions, the synthetic demo and generic replay of that generated recording. GPU checks compare depth/normals, correspondence matrices and full-volume TSDF against CPU references; exercise malformed replay, known/singular ICP, guards and behind-camera rejection. The TSDF ±0.002 mm classification band is specific to SDF = −10 mm and separately counted. Restoring the original integration kernel causes its guard regression to fail. Original ICP with finite 1.0 shared-scratch seeding fails CPU AtA comparison; the updated reduction passes with equivalent instrumentation. This is an adversarial scratch experiment, not an unmodified-runtime failure claim.

The 12-frame 193×145 moving analytic replay of three spheres in an asymmetric arrangement and a back plane ran at 512³: final pose error 2.915 mm / 0.1772°, 26,977 final raycast hits, point-to-scene RMSE 1.995 mm, same-ray range RMSE 14.414 mm and 0.43% final range outliers (>5.5 mm same-ray error). True final translation was 8.661 mm, estimated displacement 6.088 mm and stationary-pose baseline error 8.661 mm. Surface RMSE grew from 0.807 to 1.995 mm; pose error generally grew too. These are synthetic-sequence results. Retain range error because point-to-surface error alone hides visibility failures. Plane normals have measured mean interior angular error 25.09° despite matching their gradient implementation reference.

Compute Sanitizer memcheck/racecheck/initcheck/synccheck each completed 128³ component checks plus the moving demo with no reported errors/shared-memory hazards. Public evidence is updated/VALIDATION.json and updated/validation/. Ablation records retain the unmodified integration guard failure, instrumented original ICP numerical failure and instrumented fixed control pass. Outputs are PPM, ASCII PLY point samples and JSON; view_demo.py creates standalone HTML. updated/examples/demo.html and assets/updated-demo.png display actual generated-run output, distinct from historical imagery.

## Real Kinect recording

The unchanged published CUDA pipeline processed all 798 depth frames in the TUM Freiburg 1 xyz archive at full 640 × 480 resolution (26.594 seconds of capture timestamps), completing all 797 ICP transitions. Offline evaluation matched 795 motion-capture poses and rejected three timestamp associations. First-pose-anchored position RMSE was 29.501 mm versus 238.979 mm for the stationary baseline; maximum position error was 56.751 mm and last matched error 34.028 mm. Orientation RMSE was 1.200°, versus 9.986° for a stationary pose; maximum rotation from initialization was 22°. This is one recorded scene, with no surface ground truth or accuracy pass threshold; motion capture never seeds ICP.

docs/REAL_DATA.md records reproduction, conversion/evaluation and TUM CC BY 4.0 attribution. Python 3.10+, NumPy and Pillow are needed for the import tools. Preserve hashes/timestamps; use the converter's default 525/525/319.5/239.5 intrinsics and raw/5→mm units conversion without reapplying depth correction. Offline comparison uses C=diag(1,1,-1,1), anchoring at the motion-capture pose interpolated to the first depth timestamp, and no fitted trajectory transform for primary errors. The fixed 1,024 mm cube clips the larger room. Public evidence is under updated/validation/tum-fr1-xyz/.

No original GUI, live sensor SDK, optional filters, tracking recovery or performance benchmark is validated. Historical source/Source/CMakeLists.txt remains unchanged and unvalidated; the Mac CMake 4.0.3 configuration check fails because the original minimum 2.6 is below removed <3.5 compatibility.

## Maintenance workflow

Read relevant code and validation before changing claims. Preserve source provenance, authorship and terms. Review and synchronize this file and SKILL.md before every commit, including commands, outputs, status and limitations. Check Markdown, links and actual images. Do not turn component count, reference agreement or generated input into general reconstruction quality claims. Two actual Claude editorial review rounds completed on 2026-11-13 for the new port's README, build guide and context files. Useful feedback was checked against code and recorded facts, then applied. Editorial review adds no runtime evidence. A further focused Claude review completed for the real-data follow-up on 2026-11-13, checking conversion, coordinates, timing, measured errors, attribution and documentation; useful wording corrections were applied against the saved code and records.
