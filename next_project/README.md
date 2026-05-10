# UAV Formation Simulation Platform

室内多无人机编队、避障、在线重规划和容错控制实验项目。

项目主线在 `next_project/`：Python/C++/Web 三条运行线共享 `maps/`、`schemas/` 和 `outputs/` 约定。早期独立脚本保留在仓库外层 `old_code/`，论文和历史 PDF 归档在 `references/papers/`，照片重建工件归档在 `experiments/photogrammetry/`。

## Project Layout

```text
.
├── main.py              # CLI entry point
├── config.py            # simulation preset definitions
├── core/                # dynamics, control, topology, obstacles, planning, schema utils
├── simulations/         # simulation orchestration, visualization, benchmark
├── schemas/             # JSON Schema for sim_result / benchmark_result
├── scripts/             # reproduce.sh / reproduce.ps1 / compare_results.py
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

## 快速复现（≤10 行）

新机器执行以下命令即可完成 `环境检查 → pytest → 关键场景仿真 → benchmark → 汇总报告`：

```bash
# Linux / macOS
python -m pip install -e ".[dev]"
bash scripts/reproduce.sh           # 或加 --quick / --skip-tests
```

```powershell
# Windows PowerShell
python -m pip install -e ".[dev]"
powershell -ExecutionPolicy Bypass -File scripts/reproduce.ps1
```

复现产物：

- `outputs/<preset>/<run_name>/sim_result.json` — 单场景结果，遵循 `schemas/sim_result.schema.json`
- `outputs/benchmark_default/<run_name>/benchmark_results.json` — 多次评测，遵循 `schemas/benchmark_result.schema.json`
- `outputs/_reproduce/<run_name>/summary.txt` — 一站式日志

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

Available presets are defined in `config.py`. 每次运行结果默认落到
`outputs/<preset>/<timestamp>/`，可用 `--run-name` 覆盖时间戳子目录以获得稳定路径。

## Test

```bash
python -m pytest          # 串行
python -m pytest -n auto  # 需要先 pip install pytest-xdist
```

## Generated Files

Runtime figures and benchmark outputs are written to `outputs/<preset>/<timestamp>/` by default。
Each run writes a schema-validated `sim_result.json` (and benchmark scripts write
`benchmark_results.json`)，schema 详见 [`docs/benchmark-schema.md`](docs/benchmark-schema.md)。

The repository-level `.gitignore` excludes fresh outputs, Python caches, pytest caches, and C++ build products.

## Experimental Branches

照片重建是实验性支线：Web 端点 `EXPERIMENTAL /api/reconstruction/*` 只把 COLMAP/OpenMVS 输出转换成静态 `maps/*.json` 障碍物地图，不生成 `task_waypoints`，也不代表完整数字孪生。默认关闭；需要时先设置 `UAV_ENABLE_PHOTO_RECONSTRUCTION=1`，工具路径可用 `UAV_COLMAP_BIN` 和 `UAV_OPENMVS_DIR` 覆盖。

## 跨线结果对比

```bash
python scripts/compare_results.py \
    --py  outputs/basic/<ts>/sim_result.json \
    --cpp outputs/basic_cpp/<ts>/sim_result.json \
    --rel-tol 0.05 --abs-tol 0.02 \
    --report outputs/_compare/basic.md
```

## C++ Build

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build
```

The C++ targets currently include:

- `sim_main`
- `sim_benchmark`
- `sim_warehouse`
