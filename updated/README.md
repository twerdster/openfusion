# Updated OpenFusion: Linux replay and GPU checks

This directory contains the 2026 maintenance port. The recovered original remains unchanged in `../source/`, pinned at `8d435bfff054543c7e57274b773503b1370f90b4`. Derived files retain its attribution and notices. The port is separate from the students' original work and does not introduce a blanket source license.

## Implemented path

`openfusion_headless` reads calibrated binary depth frames, converts them to world-space vertices and normals on CUDA, estimates pose through ICP, integrates a TSDF volume, and raycasts its surface. ICP correspondence accumulation runs on CUDA; its 6 × 6 linear solve runs on the CPU with bundled Eigen headers. The demo generates deterministic analytic depth input and feeds the same file reader and CUDA operators used for replay.

The original OpenGL application, live camera SDKs and optional image filters are outside this target. A headless build succeeding does not establish that those paths work.

## Build and use

Use Linux, an NVIDIA GPU/driver, compatible CUDA toolkit and host compiler, CMake ≥3.18, and a C++17 compiler. Python 3 is optional for viewing saved output. No system Eigen or display libraries are required. From the repository root:

```sh
git submodule update --init
cmake -S updated -B build/updated -DCMAKE_BUILD_TYPE=Release
cmake --build build/updated --parallel 2
(cd build/updated && ctest --output-on-failure)
build/updated/openfusion_headless --demo --output openfusion-output
python3 updated/tools/view_demo.py openfusion-output/demo
```

`--self-test` runs component regressions. `--demo` runs the moving synthetic recording. With no `--self-test`, `--demo` or `--recording` option, self-test and demo both run. `--output` selects the output directory. Three CTest entries exercise component checks, the synthetic demo and file replay of the generated recording. These are not independent dataset trials. See [build and recording details](../docs/BUILD.md) for raw frame/calibration layout and the `--recording DIR --frames N` replay mode. Generic replay has no ground truth and records status `executed`, not an accuracy pass.

The default 512³ volume spans 1,024 mm at 2 mm per voxel. `-DOPENFUSION_VOLUME_SIZE=128` uses 8 mm voxels across the same volume; it is useful for smaller checks but introduces substantial quantization and banding. Resolution affects both tracking and rendering.

## Repairs and their scope

- The build uses CMake CUDA targets and a small runtime compatibility wrapper instead of the removed CUDA SDK `cutil_inline` helper. Numerical helper headers still come from the pinned source.
- TSDF integration now writes the x-plane corresponding to its world-space sample, rather than advancing the address before the first write and overrunning the final plane. It also rejects voxels behind the camera.
- ICP reduction covers the intended upper triangle, initializes symmetric entries and keeps padded image lanes in CUDA barriers. Non-multiple-of-16 image sizes are exercised.
- Matrix inverses are evaluated before assignment to avoid Eigen aliasing. ICP exposes finite/positive-definite failure and iteration status, and allocated arrays are released.
- Replay checks calibration and exact payload lengths, returns failure at end of input and rejects unsupported nonzero depth offsets. Raycast pixels that miss the volume are cleared.

The integration guard failure and instrumented reduction comparison have direct regression evidence. Explicit symmetric output is an interface contract; the original CPU solver already filled its lower triangle. Evaluated inverses, solve-status checks and cleanup are defensive correctness/maintenance changes unless separately demonstrated by a failure record. These changes do not add relocalization, loop closure or a tracking-quality guarantee.

## Checks and interpretation

The checked environment is Ubuntu 22.04, GCC 11.4, CMake 3.22.1, CUDA 11.5.119, driver 570.211.01 and an RTX 4090. Initial validation used `RelWithDebInfo`; an isolated clean `Release` build also passed the three CTest entries. [VALIDATION.json](VALIDATION.json) records the run and [validation/](validation/) retains commands and logs.

Regressions check depth/RGB byte replay, malformed inputs, depth-to-vertex and neighboring-pixel normal calculations, GPU correspondence matrices against CPU accumulation, and full-volume TSDF integration against a CPU reference. They also check known camera motion, rejection of a rank-deficient planar ICP solve, raycast geometry, allocation guard slabs and rejection of behind-camera voxels. Restoring the unmodified original integration kernel causes the allocation-guard regression to fail. The numerical reference did not reject the original reduction in the uninstrumented kernel probe; its unused shared scratch contents are undefined. Seeding its unused shared scratch with finite 1.0 values makes the original 512-entry reduction over 256 threads fail the CPU numerical reference; the updated 256-entry reduction passes with equivalent seeding. This deliberately instrumented experiment exposes dependence on scratch contents, rather than proving that every unmodified original execution fails. [Ablation records](validation/ablation-results.json) preserve both failures and the fixed control.

The TSDF reference has a separately counted ±0.002 mm band around the negative truncation boundary, SDF = −10 mm. CPU and GPU rounding can put a voxel on different sides of this cutoff. Only the corresponding weight decision in that band is treated separately and counted; this is not a general geometry tolerance. Outside the classification band, expected weights and TSDF error are checked directly. At 512³, 525 voxels were in the band and one CPU/GPU weight decision differed.

The demo uses 12 frames at 193 × 145 pixels, three spheres in an asymmetric arrangement and a back plane, with known small translation/rotation. It checks tracking against known poses and a stationary-pose baseline, volume observations/weights, per-sphere coverage and geometric error. `surface.ply` contains final raycast surface samples, not a mesh.

Two error families are reported separately. Point-to-scene error is the minimum distance to independently defined sphere/back-plane surfaces. Same-ray range error compares the raycast point with the analytic first hit for that pixel at the estimated camera pose; it captures foreground/background visibility disagreement at silhouettes. A low point-to-surface error alone cannot prove correct visibility or coverage, so range outlier and coverage checks remain part of the result.

The recorded 512³ run produced final pose error 2.915 mm / 0.1772° (also the maxima). True final translation was 8.661 mm, the estimated displacement was 6.088 mm, and the stationary-pose baseline error was 8.661 mm. Surface RMSE grew from 0.807 to 1.995 mm, and pose error generally grew across the sequence. Final same-ray range RMSE was 14.414 mm, with 0.43% of hits over the 5.5 mm threshold. This large-error tail is consistent with silhouette discrepancies; locations were not separately classified. Final raycasts hit 26,977 of 27,985 pixels, assigning 3,400 / 1,857 / 834 points within 5.5 mm of the nearest analytic sphere components.

The demo enforces the following fixture thresholds at 512³. These are regression criteria for this generated scene, not an application accuracy specification.

| Check | Required result |
| --- | --- |
| Each frame | ICP completes after the initialized first frame; pose error <12 mm and <1.5° |
| Surface geometry | RMSE <4.5 mm and 95th percentile <5.5 mm |
| Range-error tail | Fewer than 5% of hits exceed 5.5 mm same-ray error |
| Surface observations | More than half the image rays hit; each analytic sphere receives at least 100 assigned samples within 5.5 mm |
| Motion baseline | Final translation error <75% of the stationary-pose baseline; estimated displacement >50% of true final displacement |
| Integration | More than 10,000 observed voxels, maximum weight equals 12 frames, unchanged guard slabs |

Geometry/range thresholds scale with voxel width in other builds. The 128³ self-test/demo and sanitizer runs are recorded separately; generic file replay was checked at 512³. The log's expected `No transform found` message belongs to the singular-plane ICP rejection check.

Compute Sanitizer memcheck, racecheck, initcheck and synccheck each completed the 128³ component checks and moving demo with no reported errors or shared-memory race hazards. This does not establish memory/synchronization correctness for every resolution, input or application path. This initcheck version checks global initialization, not shared-memory initialization; CPU numerical references and the separately qualified scratch experiment address the ICP reduction. Saved logs identify commands and scope.

## Saved output and limitations

`demo/recording/` contains generated calibration and `uint16` depth frames. PPM files show input depth/normals and per-frame reconstruction raycasts. JSON contains errors, poses, coverage and integration counts. `view_demo.py` embeds PPM, ASCII PLY and JSON output into standalone HTML using Python's standard library. Saved-frame playback and point-cloud rotation display existing output; they do not execute CUDA in the browser.

The checked-in [viewer](examples/demo.html) and [preview image](../assets/updated-demo.png) come from the Linux `run-512/demo` output whose metrics are retained in [metrics-512.json](validation/metrics-512.json). The preview is a browser screenshot of the viewer's three saved image panels. [Artifact provenance](examples/provenance.json) records input/output hashes and reproduction commands.

Raycasting retains nearest-voxel lookup and a fixed 1.5 mm step. Its TSDF finite-difference normals had a mean interior angular error of 25.09° on the analytic plane fixture, even though they matched the CPU gradient reference. This is a geometric limitation, not a normal-quality pass. TSDF integration subtracts camera-to-sample radial ranges rather than measuring true nearest-surface distance; truncation and weight thresholds remain fixed.

No real camera recording, extended motion, tracking-loss recovery, live sensor, original GUI or throughput benchmark has been validated. Synthetic input supports independent geometric expectations and fault regression, but does not establish robustness to sensor noise, missing data or calibration error.
