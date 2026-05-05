# C++ Refactor

This directory contains the C++ port/refactor of the Python simulation code.

The C++ implementation is a runnable subset of the Python mainline. It currently covers formation dynamics, PID/SMC/backstepping control, A*/Hybrid A* obstacle planning, engineering FIRI-style convex-corridor refinement, Safe/Danger scheduling with a GNN visible-graph variant, online `WindowReplanner` task/local waypoint layering, risk-adaptive replanning support, six-direction range sensing with dynamic obstacle TTL, APF avoidance, fault injection/detection/reconfiguration, benchmark output, and SVG/PNG visualization. Python-only features at the moment include the full Python obstacle-event logging surface and Plotly HTML visualization.

## Layout

```text
cpp/
├── CMakeLists.txt
├── include/
└── src/
```

Generated C++ outputs, object files, and the previous build directory were moved to `../artifacts/cpp/`.

## Build

From the project root:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build --config Release
```

Or from inside `cpp/`:

```bash
cmake -S . -B build
cmake --build build --config Release
```

## Targets

- `sim_main`: basic formation simulation.
- `sim_benchmark`: benchmark runner.
- `sim_warehouse`: warehouse obstacle scenario.

## Manual g++ Build

If CMake is unavailable, compile from inside `cpp/`:

```bash
mkdir -p build
g++ -O3 -std=c++20 -Iinclude \
    src/rotor.cpp src/allocator.cpp src/wind_field.cpp src/smc.cpp \
    src/controller.cpp src/topology.cpp src/drone.cpp \
    src/occupancy_grid.cpp src/map_loader.cpp \
    src/firi.cpp src/visibility_graph.cpp src/gnn_planner.cpp \
    src/artificial_potential_field.cpp src/obstacle_scenario.cpp \
    src/formation_simulation.cpp src/visualization.cpp \
    src/main.cpp -o build/sim_main
```

## Run

```bash
./build/sim_main
./build/sim_benchmark
./build/sim_warehouse
```

On Windows/MSYS2, the executable names may end with `.exe`.
