from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Mapping, Sequence


MetricRow = Mapping[str, object]

FIELD_LABELS = {
    "scenario": "场景",
    "variant": "变体",
    "preset": "预设",
    "runtime_engine": "运行引擎",
    "runtime_s": "运行时间 s",
    "completed_waypoint_count": "到达航点数",
    "collision_count": "碰撞次数",
    "replan_count": "重规划次数",
    "fault_count": "故障次数",
    "planned_path_length": "规划路径长度 m",
    "executed_path_length": "执行路径长度 m",
    "trajectory_path_length": "轨迹路径长度 m",
    "trajectory_max_speed": "最大速度 m/s",
    "trajectory_max_acceleration": "最大加速度 m/s²",
    "trajectory_mean_jerk": "平均 jerk",
    "trajectory_max_jerk": "最大 jerk",
    "trajectory_jerk_squared_integral": "jerk 平方积分",
    "trajectory_snap_squared_integral": "snap 平方积分",
    "mean_error_overall": "平均误差 m",
    "max_error_overall": "最大误差 m",
    "final_error_overall": "最终误差 m",
    "min_inter_drone_distance": "最小机间距 m",
    "downwash_hits": "下洗命中数",
    "formation_clearance_mode": "安全裕度模式",
    "formation_clearance_required": "要求安全裕度 m",
    "formation_clearance_min_margin": "最小安全裕度 m",
    "formation_clearance_min_leader_margin": "leader 最小裕度 m",
    "formation_clearance_min_follower_margin": "从机最小裕度 m",
    "formation_clearance_violation_count": "安全裕度违规数",
    "formation_clearance_follower_violation_count": "从机裕度违规数",
    "formation_clearance_sample_count": "安全裕度采样数",
    "formation_clearance_posthoc_required": "post-hoc 要求裕度 m",
    "formation_clearance_posthoc_min_margin": "post-hoc 最小裕度 m",
    "formation_clearance_posthoc_violation_count": "post-hoc 违规数",
    "formation_clearance_posthoc_follower_violation_count": "post-hoc 从机违规数",
    "formation_adaptation_count": "队形调整次数",
    "formation_adaptation_last": "最终队形",
    "formation_adaptation_last_reason": "最后调整原因",
    "lookahead_reference_blocked_count": "前瞻阻断次数",
    "rrt_escape_attempt_count": "RRT escape 尝试数",
    "rrt_escape_accepted_count": "RRT escape 接受数",
    "rrt_escape_failed_count": "RRT escape 失败数",
    "clearance_warning_count": "裕度告警数",
    "clearance_warning_step_count": "裕度告警步数",
    "collision_step_count": "碰撞步数",
    "hard_collision_count": "硬碰撞数",
    "hard_collision_step_count": "硬碰撞步数",
    "min_obstacle_signed_distance": "最小障碍物有符号距离 m",
}

SCENARIO_LABELS = {
    "basic": "基础编队",
    "obstacle": "简单障碍物",
    "warehouse": "仓库离线",
    "warehouse_online": "仓库在线",
    "warehouse_danger": "仓库 Danger",
    "meeting_room": "会议室",
    "meeting_room_online": "会议室在线",
    "rrt_dual_channel_online": "RRT双通道绕行测试",
    "formation_maze_stress_online": "编队迷宫压力测试",
    "unknown_map_online": "完全未知地图在线测试",
}

VARIANT_LABELS = {
    "baseline": "基线",
    "leader_only_planner": "仅 leader 规划",
    "formation_aware_clearance": "编队感知安全裕度",
    "formation_aware_static": "编队感知固定队形",
    "formation_aware_adaptive": "编队感知自适应队形",
    "formation_aware_lookahead_adaptive": "编队感知前瞻RRT自适应",
    "no_gnn": "关闭 GNN",
    "with_trajectory_optimizer": "启用轨迹优化器",
}

VALUE_LABELS = {
    "formation_clearance_mode": {
        "leader_only_planner": "仅 leader 中心线",
        "formation_aware_clearance": "编队感知安全裕度",
    },
    "formation_adaptation_last": {
        "line": "直线队形",
        "triangle": "三角队形",
        "diamond": "菱形队形",
        "v_shape": "V形队形",
    },
    "formation_adaptation_last_reason": {
        "clearance_violation": "安全裕度不足",
        "lookahead_corridor": "前瞻通道收缩",
        "lookahead_sharp_turn": "前瞻急转弯",
        "rrt_escape": "RRT escape 可行绕行",
    },
}

VARIANT_NOTES = {
    "baseline": "默认控制与规划参数，用作基准。",
    "leader_only_planner": "只约束 leader 中心线，事后评估从机安全裕度。",
    "formation_aware_clearance": "规划阶段纳入 leader 与从机 offset 的统一安全裕度约束。",
    "formation_aware_static": "启用编队感知安全裕度，但固定初始队形。",
    "formation_aware_adaptive": "启用编队感知安全裕度，并允许队形按局部可行域自适应切换。",
    "formation_aware_lookahead_adaptive": "在自适应队形基础上加入前瞻窗口与 RRT escape，用于提前绕开急转弯或死路采样。",
    "no_gnn": "关闭 GNN 风险模块，用于观察传统规划基线。",
    "with_trajectory_optimizer": "启用高阶平滑代理目标，观察轨迹连续性指标变化。",
}


def write_summary_csv(rows: Sequence[MetricRow], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows)
    with target.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})
    return target


def write_markdown_report(rows: Sequence[MetricRow], path: str | Path, *, title: str = "消融对比报告") -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows)
    lines = [f"# {title}", ""]
    if not rows:
        lines.append("无数据。")
    else:
        lines.extend(_settings_section(rows))
        lines.extend(["", "## 快速消融结果", ""])
        lines.append("| " + " | ".join(_label_for_field(key) for key in fieldnames) + " |")
        lines.append("| " + " | ".join("---" for _ in fieldnames) + " |")
        for row in rows:
            lines.append("| " + " | ".join(_format_cell(_display_value(key, row.get(key, ""))) for key in fieldnames) + " |")
        lines.extend(["", "## 结论", ""])
        lines.extend(_interpretation_lines(rows))
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target


def _fieldnames(rows: Sequence[MetricRow]) -> list[str]:
    preferred = [
        "scenario",
        "variant",
        "preset",
        "runtime_engine",
        "runtime_s",
        "completed_waypoint_count",
        "collision_count",
        "replan_count",
        "fault_count",
        "planned_path_length",
        "executed_path_length",
        "trajectory_path_length",
        "trajectory_max_speed",
        "trajectory_max_acceleration",
        "trajectory_mean_jerk",
        "trajectory_max_jerk",
        "trajectory_jerk_squared_integral",
        "trajectory_snap_squared_integral",
        "mean_error_overall",
        "max_error_overall",
        "final_error_overall",
        "min_inter_drone_distance",
        "downwash_hits",
        "formation_clearance_mode",
        "formation_clearance_required",
        "formation_clearance_min_margin",
        "formation_clearance_min_leader_margin",
        "formation_clearance_min_follower_margin",
        "formation_clearance_violation_count",
        "formation_clearance_follower_violation_count",
        "formation_clearance_sample_count",
        "formation_clearance_posthoc_required",
        "formation_clearance_posthoc_min_margin",
        "formation_clearance_posthoc_violation_count",
        "formation_clearance_posthoc_follower_violation_count",
        "formation_adaptation_count",
        "formation_adaptation_last",
        "formation_adaptation_last_reason",
        "lookahead_reference_blocked_count",
        "rrt_escape_attempt_count",
        "rrt_escape_accepted_count",
        "rrt_escape_failed_count",
    ]
    keys = set()
    for row in rows:
        keys.update(str(key) for key in row.keys())
    return [key for key in preferred if key in keys] + sorted(keys.difference(preferred))


def _format_cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, str):
        text = value.strip()
        if text:
            try:
                number = float(text)
            except ValueError:
                pass
            else:
                if math.isfinite(number):
                    return f"{number:.6g}"
    return str(value).replace("|", "\\|")


def _label_for_field(key: str) -> str:
    return FIELD_LABELS.get(key, key)


def _display_value(key: str, value: object) -> object:
    if isinstance(value, str):
        if key == "scenario":
            return SCENARIO_LABELS.get(value, value)
        if key == "variant":
            return VARIANT_LABELS.get(value, value)
        return VALUE_LABELS.get(key, {}).get(value, value)
    return value


def _settings_section(rows: Sequence[MetricRow]) -> list[str]:
    lines = [
        "## 消融设置",
        "",
        "| 场景 | 变体 | 说明 |",
        "| --- | --- | --- |",
    ]
    seen: set[tuple[str, str]] = set()
    for row in rows:
        scenario = str(row.get("scenario", ""))
        variant = str(row.get("variant", ""))
        key = (scenario, variant)
        if key in seen:
            continue
        seen.add(key)
        lines.append(
            "| "
            + " | ".join(
                [
                    _format_cell(_display_value("scenario", scenario)),
                    _format_cell(_display_value("variant", variant)),
                    _format_cell(VARIANT_NOTES.get(variant, "保留原始参数，用于横向比较。")),
                ]
            )
            + " |"
        )
    return lines


def _interpretation_lines(rows: Sequence[MetricRow]) -> list[str]:
    lines = [
        "- 先看 `碰撞次数`、`硬碰撞数` 与 `到达航点数`，确认方案是否满足基本安全与任务完成要求。",
        "- 再看 `post-hoc 从机违规数`、`从机最小裕度 m` 和 `最小机间距 m`，判断 leader 安全是否真正扩展到整队安全。",
        "- 最后结合 `队形调整次数`、`前瞻阻断次数` 与 `RRT escape 接受数`，解释自适应队形和局部绕行是否实际参与决策。",
    ]
    if any(float(row.get("rrt_escape_accepted_count") or 0) > 0 for row in rows):
        lines.append("- 本轮存在被接受的 RRT escape，说明采样到死路或急转弯时已使用全局/局部绕行候选辅助队形决策。")
    return lines


__all__ = ["write_markdown_report", "write_summary_csv"]
