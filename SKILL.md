# Maintaining openfusion

Read SUMMARY.md, then this file, then README.md. Inspect relevant code, original usage records and any updated/VALIDATION.json before changing technical claims.

## Repository workflow

Clone recursively. source/Source/CMakeLists.txt is entrypoint; docs/BUILD.md documents modern invocations of original CMake/backend options and blockers. Replay reads Parameters.txt and recordedN/recorded-rgbN binary files.

Treat these commands as scoped component or historical inspection procedures exactly as documented. No build/package step bundles this repository's wrapper documentation. Do not rerun missing-dependency/full-hardware applications merely to verify prose edits. When code changes, run the affected component checks and record actual environment, skips and limits.

## Preservation and claims

Keep original source bytes, submodule pins, provenance manifests, student/third-party credits and licenses intact. Do not edit upstream README files in submodules or historical source documentation to modernize the wrapper. Put any future implementation or dependency adaptation in a separately labeled updated/ area with its own tests.

Requires compatible NVIDIA CUDA, Eigen/CImg/GLEW/GLUT/OpenGL/X11/pthreads and selected sensor SDK. Legacy CUDA utility headers/compute targets and hardcoded paths require work. No updated port/notebook. No overall source license file found; individual terms remain.

Before each commit, review and synchronize SUMMARY.md and this file. Validate Markdown/relative links, retained figure paths and diff scope. Keep historical figures distinct from current output, and synthetic examples distinct from measurements. Explain what unit-test counts cover; do not convert source inspection or failed builds into compatibility, correctness or benchmark claims. Do not claim an external review unless it actually occurred.
