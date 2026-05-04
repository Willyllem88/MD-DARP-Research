# Study of Solution Methods for the Ride-Sharing Problem

The aim is to develop different methods to solve the RSP, such as mathematical models, heuristics, and metaheuristics. It also intends to explore how the RSP can be extended to a dynamic version.

- Author: GUILLEM CABRÉ FARRÉ
- Director: PEDRO JESÚS COPADO MÉNDEZ
- Codirector: CAROLINE KÖNIG

## Usage

```bash
mkdir build
cd build
cmake ..
cmake --build . -j
./cpp/core/darpmd_runner --help
```

## Cordeau DARP Instances

To install the Cordeau DARP instances (A-series(24) + B-series(24) + R-series(20) = 68 instances), run the following command from the root of the project:

```bash
cd build
cmake ..
cmake --build . --target install_cordeau_instances
```
