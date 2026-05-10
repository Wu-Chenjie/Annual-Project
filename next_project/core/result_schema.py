"""统一结果 schema 工具。

提供：
- ``SCHEMA_VERSION``：当前 schema 语义版本。
- ``load_schema(name)``：按名加载 ``schemas/`` 下的 JSON Schema。
- ``validate(payload, name)``：用 ``jsonschema``（如已安装）做严格校验，否则回退到一份轻量内置检查。
- ``json_safe(obj)``：递归把 numpy/标量结构变成可 JSON 序列化对象。
- ``build_sim_result_payload(...)`` / ``build_benchmark_payload(...)``：
  将仿真/基准的原始结果包装为符合 schema 的字典。

设计目标：
- 让 Python、C++（通过 nlohmann/json）、Web 三条运行线共享同一份字段约定；
- Python 侧默认开启校验，schema 不匹配立即抛错，避免"字段漂移"。
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np


SCHEMA_VERSION = "1.0.0"

_SCHEMA_DIR = Path(__file__).resolve().parents[1] / "schemas"


# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------


def json_safe(obj: Any) -> Any:
    """递归把 numpy / dataclass 等转成可 ``json.dumps`` 的 Python 原生类型。"""
    if obj is None:
        return None
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, Mapping):
        return {str(k): json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [json_safe(v) for v in obj]
    return obj


def utc_now_iso() -> str:
    """返回 ISO-8601 UTC 时间戳（精确到秒）。"""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


# ---------------------------------------------------------------------------
# Schema 加载与校验
# ---------------------------------------------------------------------------


def load_schema(name: str) -> dict[str, Any]:
    """按名加载 schema。``name`` 可以是 ``benchmark_result`` 或 ``sim_result``。"""
    candidates = [
        _SCHEMA_DIR / f"{name}.schema.json",
        _SCHEMA_DIR / name,
    ]
    for path in candidates:
        if path.is_file():
            return json.loads(path.read_text(encoding="utf-8"))
    raise FileNotFoundError(f"未找到 schema: {name}（搜索目录：{_SCHEMA_DIR}）")


def _fallback_validate(payload: Any, schema: Mapping[str, Any], pointer: str = "") -> list[str]:
    """轻量校验：仅覆盖 type/required/enum/pattern 等核心约束，作为 jsonschema 不可用时的回退。"""
    errors: list[str] = []
    expected = schema.get("type")
    if expected is not None:
        if isinstance(expected, list):
            ok = any(_match_type(payload, t) for t in expected)
        else:
            ok = _match_type(payload, expected)
        if not ok:
            errors.append(f"{pointer or '/'}: 期望类型 {expected}，实际 {type(payload).__name__}")
            return errors

    if isinstance(payload, Mapping):
        required = schema.get("required") or []
        for key in required:
            if key not in payload:
                errors.append(f"{pointer}/{key}: 必填字段缺失")
        properties = schema.get("properties") or {}
        for key, sub in properties.items():
            if key in payload:
                errors.extend(_fallback_validate(payload[key], sub, f"{pointer}/{key}"))

    if isinstance(payload, list):
        items = schema.get("items")
        if isinstance(items, Mapping):
            for idx, item in enumerate(payload):
                errors.extend(_fallback_validate(item, items, f"{pointer}[{idx}]"))

    enum = schema.get("enum")
    if enum is not None and payload not in enum:
        errors.append(f"{pointer or '/'}: {payload!r} 不在枚举 {enum} 中")

    return errors


def _match_type(value: Any, expected: str) -> bool:
    if expected == "object":
        return isinstance(value, Mapping)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "integer":
        return isinstance(value, bool) is False and isinstance(value, int)
    if expected == "number":
        return isinstance(value, bool) is False and isinstance(value, (int, float))
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "null":
        return value is None
    return True


def validate(payload: Mapping[str, Any], schema_name: str, *, strict: bool = True) -> list[str]:
    """校验 ``payload`` 是否符合 schema。

    返回错误信息列表，空列表表示通过。``strict=True`` 时检测到错误会直接抛
    ``ValueError``，方便测试代码 ``assert validate(...) == []`` 之外的快速失败。
    """
    schema = load_schema(schema_name)
    errors: list[str]
    try:
        import jsonschema  # type: ignore

        validator = jsonschema.Draft7Validator(schema)
        errors = [
            f"{'/'.join(str(p) for p in err.absolute_path) or '/'}: {err.message}"
            for err in validator.iter_errors(payload)
        ]
    except Exception:  # pragma: no cover - 依赖缺失或异常时回退
        errors = _fallback_validate(payload, schema)

    if strict and errors:
        raise ValueError(
            f"schema 校验失败 ({schema_name}):\n  - " + "\n  - ".join(errors)
        )
    return errors


# ---------------------------------------------------------------------------
# Payload 构造
# ---------------------------------------------------------------------------


def _engine_version() -> str:
    return os.environ.get("UAV_ENGINE_VERSION", "0.1.0")


def _per_follower(values: Iterable[Any]) -> list[float]:
    return [float(v) for v in values]


def build_sim_result_payload(
    *,
    preset: str,
    sim_result: Mapping[str, Any],
    runtime_s: float | None,
    runtime_engine: str = "python",
    config_snapshot: Mapping[str, Any] | None = None,
    include_trajectories: bool = False,
    risk_report: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """把 ``FormationSimulation.run()`` / ``ObstacleScenarioSimulation.run()`` 的输出包装为 schema 标准格式。"""
    metrics = sim_result.get("metrics") or {}
    means = _per_follower(metrics.get("mean", []))
    maxs = _per_follower(metrics.get("max", []))
    finals = _per_follower(metrics.get("final", []))

    summary = {
        "mean_error_overall": float(np.mean(means)) if means else 0.0,
        "max_error_overall": float(np.max(maxs)) if maxs else 0.0,
        "final_error_overall": float(np.mean(finals)) if finals else 0.0,
        "collision_count": len(sim_result.get("collision_log") or []),
        "replan_count": len(sim_result.get("replan_events") or []),
        "fault_count": len(sim_result.get("fault_log") or []),
    }

    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "preset": preset,
        "runtime_engine": runtime_engine,
        "engine_version": _engine_version(),
        "generated_at": utc_now_iso(),
        "metrics": {
            "mean": means,
            "max": maxs,
            "final": finals,
        },
        "completed_waypoint_count": int(sim_result.get("completed_waypoint_count") or 0),
        "summary": summary,
    }

    if runtime_s is not None:
        payload["runtime_s"] = float(runtime_s)
    if config_snapshot is not None:
        payload["config_snapshot"] = json_safe(dict(config_snapshot))

    optional_passthrough = (
        "planned_path",
        "executed_path",
        "replan_events",
        "collision_log",
        "fault_log",
        "sensor_logs",
        "safety_metrics",
    )
    for key in optional_passthrough:
        if key in sim_result:
            payload[key] = json_safe(sim_result[key])

    if include_trajectories:
        trajectories: dict[str, Any] = {}
        if "time" in sim_result:
            trajectories["time"] = json_safe(sim_result["time"])
        if "leader" in sim_result:
            trajectories["leader"] = json_safe(sim_result["leader"])
        if "followers" in sim_result:
            trajectories["followers"] = json_safe(sim_result["followers"])
        if trajectories:
            payload["trajectories"] = trajectories

    if risk_report is not None:
        payload["risk_report"] = json_safe(dict(risk_report))

    return payload


def build_benchmark_payload(
    *,
    preset: str,
    runs: int,
    records: list[Mapping[str, Any]],
    summary: Mapping[str, Any],
    runtime_engine: str = "python",
    config_template: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """把 benchmark 多次运行的统计结果包装为 schema 标准格式。"""
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "preset": preset,
        "runtime_engine": runtime_engine,
        "engine_version": _engine_version(),
        "generated_at": utc_now_iso(),
        "summary": json_safe(dict(summary)),
        "records": [json_safe(dict(r)) for r in records],
    }
    if config_template is not None:
        payload["config_template"] = json_safe(dict(config_template))
    payload["summary"].setdefault("runs", runs)
    return payload


def build_web_sim_result_payload(
    *,
    preset: str,
    web_results: Mapping[str, Any] | None = None,
    runtime_s: float | None = None,
) -> dict[str, Any]:
    """Web ``/api/simulate`` 响应的 schema 包装。

    原 C++ ``sim_dynamic_replay`` 输出嵌套在 ``results`` 字段中，顶层字段遵循
    ``sim_result`` 和 ``benchmark_result`` 的共通约定。
    不要求 Web 侧提供完整 ``metrics`` / ``summary``，因为重放输出可能只有轨迹数据。"""
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "preset": preset,
        "runtime_engine": "web",
        "engine_version": _engine_version(),
        "generated_at": utc_now_iso(),
        "metrics": {
            "mean": [],
            "max": [],
            "final": [],
        },
        "completed_waypoint_count": 0,
        "summary": {
            "mean_error_overall": 0.0,
            "max_error_overall": 0.0,
            "final_error_overall": 0.0,
            "collision_count": 0,
            "replan_count": 0,
            "fault_count": 0,
        },
    }
    if runtime_s is not None:
        payload["runtime_s"] = float(runtime_s)
    if web_results is not None:
        payload["results"] = json_safe(dict(web_results))
        if isinstance(web_results.get("summary"), Mapping):
            summary = web_results["summary"]
            payload["summary"]["collision_count"] = int(summary.get("collision_count") or 0)
            payload["summary"]["replan_count"] = int(summary.get("replan_count") or 0)
            payload["summary"]["fault_count"] = int(summary.get("fault_count") or 0)
    return payload


def write_json(payload: Mapping[str, Any], path: str | Path, *, indent: int = 2) -> Path:
    """带父目录创建的 JSON 落盘。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload, ensure_ascii=False, indent=indent),
        encoding="utf-8",
    )
    return target


__all__ = [
    "SCHEMA_VERSION",
    "build_benchmark_payload",
    "build_sim_result_payload",
    "build_web_sim_result_payload",
    "json_safe",
    "load_schema",
    "utc_now_iso",
    "validate",
    "write_json",
]


if __name__ == "__main__":  # pragma: no cover - 开发时手动校验
    if len(sys.argv) < 3:
        print("usage: python -m core.result_schema <schema_name> <payload.json>")
        sys.exit(2)
    schema_name = sys.argv[1]
    data = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    errs = validate(data, schema_name, strict=False)
    if errs:
        print("FAIL")
        for e in errs:
            print(" -", e)
        sys.exit(1)
    print("OK")
