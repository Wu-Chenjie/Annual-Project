#!/usr/bin/env bash
# scripts/reproduce.sh
# 一键复现：环境检查 → 全量 pytest → 关键场景仿真 → benchmark → 结果汇总。
# 用法:
#   bash scripts/reproduce.sh                # 默认全流程
#   bash scripts/reproduce.sh --skip-tests   # 跳过 pytest
#   bash scripts/reproduce.sh --quick        # 仅跑 1 个代表场景 + 1 次 benchmark
#
# 设计原则：
# - 任意一步失败立即退出，并回打 summary。
# - 所有运行产物落到 outputs/<preset>/<run_name>/，每次运行使用同一个时间戳。

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

SKIP_TESTS=0
QUICK=0
for arg in "$@"; do
  case "$arg" in
    --skip-tests) SKIP_TESTS=1 ;;
    --quick)      QUICK=1 ;;
    -h|--help)
      sed -n '2,12p' "$0"; exit 0 ;;
    *)
      echo "[reproduce] 未识别参数: $arg" >&2; exit 2 ;;
  esac
done

RUN_NAME="$(date +%Y%m%d-%H%M%S)"
SUMMARY_DIR="outputs/_reproduce/$RUN_NAME"
mkdir -p "$SUMMARY_DIR"
SUMMARY_FILE="$SUMMARY_DIR/summary.txt"

if [[ $QUICK -eq 1 ]]; then
  PRESETS=(basic)
  BENCH_RUNS=1
else
  PRESETS=(basic warehouse_danger school_corridor_online)
  BENCH_RUNS=3
fi

log()  { printf '[reproduce] %s\n' "$*" | tee -a "$SUMMARY_FILE"; }
fail() { printf '[reproduce][FAIL] %s\n' "$*" | tee -a "$SUMMARY_FILE" >&2; exit 1; }

log "PROJECT_ROOT = $PROJECT_ROOT"
log "RUN_NAME     = $RUN_NAME"

# ---- 1) 环境检查 ----
log "step 1/4: 环境检查"
python -c "import sys, numpy, scipy, matplotlib; print('python', sys.version.split()[0]); print('numpy', numpy.__version__); print('scipy', scipy.__version__)" \
  | tee -a "$SUMMARY_FILE" || fail "Python 依赖缺失，请先 pip install -r requirements.txt"

# ---- 2) 全量测试 ----
if [[ $SKIP_TESTS -eq 0 ]]; then
  log "step 2/4: pytest"
  python -m pytest -q | tee -a "$SUMMARY_FILE"
else
  log "step 2/4: pytest (skipped)"
fi

# ---- 3) 关键场景仿真 ----
log "step 3/4: 关键场景仿真 (presets: ${PRESETS[*]})"
for preset in "${PRESETS[@]}"; do
  log "  -> python main.py --preset $preset --run-name $RUN_NAME --no-plot"
  python main.py \
    --preset "$preset" \
    --run-name "$RUN_NAME" \
    --no-plot \
    | tee -a "$SUMMARY_FILE"
done

# ---- 4) benchmark ----
log "step 4/4: benchmark (runs=$BENCH_RUNS)"
python -c "
from simulations.benchmark import run_benchmark
import json, pathlib
out = pathlib.Path('outputs/benchmark_default/$RUN_NAME/benchmark_results.json')
res = run_benchmark(runs=$BENCH_RUNS, output_file=str(out), preset='benchmark_default')
print('benchmark wrote:', out)
print('worst_case_max_error =', res['summary']['worst_case_max_error'])
" | tee -a "$SUMMARY_FILE"

# ---- 汇总 ----
log "全部完成。结果根目录:"
log "  outputs/<preset>/$RUN_NAME/sim_result.json"
log "  outputs/benchmark_default/$RUN_NAME/benchmark_results.json"
log "  $SUMMARY_FILE"
