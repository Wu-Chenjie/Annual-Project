# UAV Formation Simulation Platform

室内多无人机编队、避障、在线重规划和容错控制实验项目。

## Project Layout

```text
.
├── main.py              # CLI entry point
├── config.py            # simulation preset definitions
├── core/                # dynamics, control, topology, obstacles, planning
├── simulations/         # simulation orchestration, visualization, benchmark
├── maps/                # indoor scenario JSON files
├── tests/               # pytest tests and regression checks
├── docs/                # design notes and technical documents
├── cpp/                 # C++ port / refactor
└── artifacts/           # archived generated outputs and caches
```

## Setup

```bash
python -m pip install -r requirements.txt
```

For editable development:

```bash
python -m pip install -e ".[dev]"
```

## Run

Default preset:

```bash
python main.py
```

Choose a preset:

```bash
python main.py --preset school_corridor_online
python main.py --preset meeting_room --output-dir outputs/meeting_room
python main.py --preset warehouse_danger --max-sim-time 10 --no-plot
```

Available presets are defined in `config.py`.

## Test

```bash
python -m pytest
```

## Generated Files

Runtime figures and benchmark outputs are written to `outputs/` by default. Previous generated files and caches were moved to `artifacts/` so the source tree stays readable.

The repository-level `.gitignore` excludes fresh outputs, Python caches, pytest caches, and C++ build products.

## C++ Build

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build
```

The C++ targets currently include:

- `sim_main`
- `sim_benchmark`
- `sim_warehouse`
