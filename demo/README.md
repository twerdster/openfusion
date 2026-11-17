# Updated interactive demo

Open https://twerdster.github.io/openfusion/ or serve this directory locally with `python3 -m http.server 8000 --directory demo` from the repository root. Open http://localhost:8000. No package installation or build is required. The demo uses only local static assets; the algorithm and data scope are labeled on the page.

`provenance.json` binds displayed data to recorded source artifacts. Browser interaction checks cover loading, controls, rendering, responsive layout and absence of runtime errors; they add no native algorithm, hardware or accuracy coverage.

[Recorded browser checks and scope](validation.json). The page was checked in current Chrome; physical mobile browsers and other browser engines were not tested.
