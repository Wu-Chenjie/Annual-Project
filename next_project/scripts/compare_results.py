"""scripts/compare_results.py — 跨语言结果对比脚本。

用途
----
对比 Python 与 C++ 两条运行线（或 Python 与 Web）输出的 ``sim_result.json``
和 ``benchmark_results.json``。两份文件必须符合 ``schemas/sim_result.schema.json``
或 ``schemas/benchmark_result.schema.json``。

输出
----
- 终端打印逐字段差异
- 可选：写一份 Markdown 报告

用法
----
    python scripts/compare_results.py --py outputs/basic/<ts>/sim_result.json \
                                      --cpp outputs/basic_cpp/<ts>/sim_result.json
    python scripts/compare_results.py --py a.json --cpp b.json --kind benchmark \
                                      --report outputs/_compare/report.md

判定准则
--------
- ``metrics.{mean,max,final}``、``summary.*_error_*``：相对误差 < ``--rel-tol``（默认 5%）
  且绝对误差 < ``--abs-tol``（默认 0.02 m）。
- 计数字段（``collision_count``、``replan_count`` 等）：要求完全相等。
- 任一不通过：以非零状态码退出。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# 让脚本既能 ``python scripts/compare_results.py`` 直接跑，也能作为模块。
_THIS_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _THIS_DIR.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from core.result_schema import validate  # noqa: E402


# ---------------------------------------------------------------------------
# 字段比对
# ---------------------------------------------------------------------------


def _is_close(a: float, b: float, rel_tol: float, abs_tol: float) -> bool:
    if a == b:
        return True
    diff = abs(float(a) - float(b))
    if diff <= abs_tol:
        return True
    denom = max(abs(float(a)), abs(float(b)), 1e-12)
    return diff / denom <= rel_tol


def _cmp_scalar(name: str, a: Any, b: Any, rel_tol: float, abs_tol: float, exact: bool = False) -> dict:
    if a is None or b is None:
        return {"name": name, "ok": a == b, "left": a, "right": b, "note": "one side missing"}
    if exact:
        return {"name": name, "ok": a == b, "left": a, "right": b}
    ok = _is_close(float(a), float(b), rel_tol, abs_tol)
    return {
        "name": name,
        "ok": ok,
        "left": float(a),
        "right": float(b),
        "abs_diff": abs(float(a) - float(b)),
    }


def _cmp_list(name: str, a: list, b: list, rel_tol: float, abs_tol: float) -> list[dict]:
    rows: list[dict] = []
    n = max(len(a or []), len(b or []))
    for i in range(n):
        av = a[i] if a and i < len(a) else None
        bv = b[i] if b and i < len(b) else None
        rows.append(_cmp_scalar(f"{name}[{i}]", av, bv, rel_tol, abs_tol))
    return rows


def compare_sim_results(left: dict, right: dict, rel_tol: float, abs_tol: float) -> list[dict]:
    rows: list[dict] = []
    rows.append(_cmp_scalar("preset", left.get("preset"), right.get("preset"), 0, 0, exact=True))
    rows.append(_cmp_scalar("schema_version", left.get("schema_version"), right.get("schema_version"), 0, 0, exact=True))

    lm = left.get("metrics", {})
    rm = right.get("metrics", {})
    for key in ("mean", "max", "final"):
        rows.extend(_cmp_list(f"metrics.{key}", lm.get(key, []), rm.get(key, []), rel_tol, abs_tol))

    ls = left.get("summary", {})
    rs = right.get("summary", {})
    for key in ("mean_error_overall", "max_error_overall", "final_error_overall"):
        rows.append(_cmp_scalar(f"summary.{key}", ls.get(key), rs.get(key), rel_tol, abs_tol))
    for key in ("collision_count", "replan_count", "fault_count"):
        rows.append(_cmp_scalar(f"summary.{key}", ls.get(key), rs.get(key), 0, 0, exact=True))

    rows.append(_cmp_scalar(
        "completed_waypoint_count",
        left.get("completed_waypoint_count"),
        right.get("completed_waypoint_count"),
        0, 0, exact=True,
    ))
    return rows


def compare_benchmark(left: dict, right: dict, rel_tol: float, abs_tol: float) -> list[dict]:
    rows: list[dict] = []
    rows.append(_cmp_scalar("preset", left.get("preset"), right.get("preset"), 0, 0, exact=True))
    rows.append(_cmp_scalar("runs", left.get("summary", {}).get("runs"), right.get("summary", {}).get("runs"), 0, 0, exact=True))

    ls = left.get("summary", {})
    rs = right.get("summary", {})
    for key in ("runtime_mean_s", "runtime_std_s", "worst_case_max_error"):
        rows.append(_cmp_scalar(f"summary.{key}", ls.get(key), rs.get(key), rel_tol, abs_tol))
    for key in ("mean_error_mean", "mean_error_std", "max_error_mean", "max_error_std"):
        rows.extend(_cmp_list(f"summary.{key}", ls.get(key, []), rs.get(key, []), rel_tol, abs_tol))
    rows.append(_cmp_scalar(
        "summary.all_runs_pass_0_3m",
        ls.get("all_runs_pass_0_3m"),
        rs.get("all_runs_pass_0_3m"),
        0, 0, exact=True,
    ))
    return rows


# ---------------------------------------------------------------------------
# 报告输出
# ---------------------------------------------------------------------------


def _format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def render_markdown(rows: list[dict], left_label: str, right_label: str, kind: str) -> str:
    lines = [
        f"# Cross-line comparison ({kind})",
        "",
        f"- left:  `{left_label}`",
        f"- right: `{right_label}`",
        "",
        "| field | left | right | abs_diff | ok |",
        "| --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        lines.append(
            "| {name} | {left} | {right} | {diff} | {ok} |".format(
                name=row["name"],
                left=_format_value(row.get("left")),
                right=_format_value(row.get("right")),
                diff=_format_value(row.get("abs_diff")) if "abs_diff" in row else "",
                ok="✅" if row["ok"] else "❌",
            )
        )
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _load_and_validate(path: Path, schema_name: str, *, no_validate: bool) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not no_validate:
        validate(payload, schema_name, strict=True)
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--py", dest="left", required=True, help="Python 输出 JSON 路径（视作 left）")
    parser.add_argument("--cpp", dest="right", required=True, help="C++ 或其他运行线 JSON 路径（视作 right）")
    parser.add_argument("--kind", choices=("sim", "benchmark"), default="sim", help="对比对象类型")
    parser.add_argument("--rel-tol", type=float, default=0.05, help="数值字段相对误差上限")
    parser.add_argument("--abs-tol", type=float, default=0.02, help="数值字段绝对误差上限（米/秒）")
    parser.add_argument("--report", default=None, help="可选：将 markdown 报告写到该路径")
    parser.add_argument("--no-validate", action="store_true", help="跳过 JSON Schema 校验")
    args = parser.parse_args(argv)

    schema_name = "sim_result" if args.kind == "sim" else "benchmark_result"
    left = _load_and_validate(Path(args.left), schema_name, no_validate=args.no_validate)
    right = _load_and_validate(Path(args.right), schema_name, no_validate=args.no_validate)

    if args.kind == "sim":
        rows = compare_sim_results(left, right, args.rel_tol, args.abs_tol)
    else:
        rows = compare_benchmark(left, right, args.rel_tol, args.abs_tol)

    failures = [r for r in rows if not r["ok"]]

    md = render_markdown(rows, args.left, args.right, args.kind)
    print(md)

    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(md, encoding="utf-8")
        print(f"report written to {report_path}")

    if failures:
        print(f"FAIL: {len(failures)} fields out of tolerance", file=sys.stderr)
        return 1
    print(f"PASS: {len(rows)} fields within tolerance")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
