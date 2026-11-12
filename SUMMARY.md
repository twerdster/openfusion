# openfusion: maintenance context

Read this file first, then SKILL.md and README.md. This context describes the repository as of 2026-11-12.

## Purpose and current source

C++/CUDA depth reconstruction: vertex/normal maps, ICP alignment, TSDF volume integration and raycasting.

source/ pins gilbs10/OpenFusion at 8d435bfff054543c7e57274b773503b1370f90b4 without modifications; report title page credits Nurit Schwabsky/Vered Cohen and supervisors Prof. Chaim Gotsman/Aaron Wetzler. The owner profile identifies gilbs10 as Gil Ben-Shachar. No Visual Studio solution is present in the pinned revision. Camera abstraction includes file, PrimeSense/OpenNI, SoftKinetic/DepthSense and GIP camera backends, not arbitrary-camera compatibility.

## Use and validation

Clone recursively. source/Source/CMakeLists.txt is entrypoint; docs/BUILD.md documents modern invocations of original CMake/backend options and blockers. Replay reads Parameters.txt and recordedN/recorded-rgbN binary files.

Recorded local configuration failed before compilation at obsolete CMake 2.6 compatibility. No successful modern build, numerical comparison, sensor runtime or benchmark. GPU/legacy SDKs/recordings unavailable.

Requires compatible NVIDIA CUDA, Eigen/CImg/GLEW/GLUT/OpenGL/X11/pthreads and selected sensor SDK. Legacy CUDA utility headers/compute targets and hardcoded paths require work. No updated port/notebook. No overall source license file found; individual terms remain.

## Documentation maintenance

The 2026-11-12 documentation revision organizes the README around implementation, available source, setup, recorded checks and limits. It preserves historical visuals and links, with captions separating historical results from updated examples. No runnable code, source provenance, pinned revisions, licenses or validation results changed in this revision. Recorded earlier tests were not rerun for the prose-only edits.

Preserve original source and authorship. New ports belong under updated/ and require their own scope/validation. Do not infer a blanket license from recovery. Review this file and SKILL.md before every commit and synchronize changes to workflow, commands, validation or status. Check relative links and figure rendering; never describe inventory counts or component tests as full-application validation.

Two actual Claude editorial review rounds on 2026-11-12 covered the README and related changed documentation, including context files, component guides and applicable algorithm/build notes or schematic text. Useful qualifications were checked against source, preserved project records and saved validation logs, then applied. Editorial review adds no runtime coverage.
