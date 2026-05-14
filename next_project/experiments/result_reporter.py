from __future__ import annotations

import json
import warnings
from pathlib import Path
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np

from experiments.metrics_extractor import extract_metrics

plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "Arial Unicode MS",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


FIGURE_NAMES = {
    "route": "场景与路线.png",
    "route_3d": "三维飞行轨迹.png",
    "flight": "飞行参数.png",
    "errors": "误差统计.png",
    "events": "飞行事件时间线.png",
    "planning": "规划器耗时.png",
}

PRESET_LABELS = {
    "unknown_map_online": "完全未知地图在线测试",
    "obstacle_unknown": "简单障碍物完全未知",
    "warehouse_unknown": "工业仓库完全未知",
    "warehouse_a_unknown": "仓库 A* 完全未知",
    "warehouse_online_unknown": "仓库在线完全未知",
    "warehouse_danger_unknown": "仓库 Danger 完全未知",
    "fault_tolerance_unknown": "容错测试完全未知",
    "fault_tolerance_online_unknown": "容错在线完全未知",
    "school_corridor_unknown": "学校走廊完全未知",
    "school_corridor_online_unknown": "学校走廊在线完全未知",
    "company_cubicles_unknown": "公司格子间完全未知",
    "company_cubicles_online_unknown": "公司格子间在线完全未知",
    "meeting_room_unknown": "会议室完全未知",
    "meeting_room_online_unknown": "会议室在线完全未知",
    "rrt_dual_channel_online_unknown": "RRT 双通道绕行完全未知",
    "formation_maze_stress_online_unknown": "编队迷宫压力完全未知",
    "laboratory_unknown": "实验室完全未知",
    "laboratory_online_unknown": "实验室在线完全未知",
}


def generate_result_report(
    input_json: str | Path,
    output_dir: str | Path | None = None,
    *,
    title: str | None = None,
    report_name: str = "report.md",
    metrics_name: str = "metrics.json",
    figure_dir_name: str = "report_figures",
) -> Path:
    source = Path(input_json)
    payload = json.loads(source.read_text(encoding="utf-8"))
    target_dir = Path(output_dir) if output_dir is not None else source.parent
    target_dir.mkdir(parents=True, exist_ok=True)
    figure_dir = target_dir / figure_dir_name
    figure_dir.mkdir(parents=True, exist_ok=True)

    metrics = extract_metrics(payload)
    (target_dir / metrics_name).write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    figures = {
        "route": _plot_route(payload, figure_dir / FIGURE_NAMES["route"]),
        "route_3d": _plot_route_3d(payload, figure_dir / FIGURE_NAMES["route_3d"]),
        "flight": _plot_flight_parameters(payload, metrics, figure_dir / FIGURE_NAMES["flight"]),
        "errors": _plot_tracking_errors(payload, figure_dir / FIGURE_NAMES["errors"]),
        "events": _plot_event_timeline(payload, figure_dir / FIGURE_NAMES["events"]),
        "planning": _plot_planning_times(payload, figure_dir / FIGURE_NAMES["planning"]),
    }

    report_title = title or _default_report_title(payload)
    report_path = target_dir / report_name
    report_path.write_text(
        _render_markdown(payload, metrics, report_title, figures, figure_dir_name=figure_dir_name),
        encoding="utf-8",
    )
    return report_path


def _render_markdown(
    payload: Mapping[str, Any],
    metrics: Mapping[str, Any],
    title: str,
    figures: Mapping[str, Path],
    *,
    figure_dir_name: str = "report_figures",
) -> str:
    cfg = dict(payload.get("config_snapshot") or {})
    if payload.get("runtime_engine") == "cpp":
        cfg.setdefault("planner_kind", "cpp")
        cfg.setdefault("planner_mode", "cpp")
    planned_trajectory = payload.get("planned_trajectory") or {}
    trajectory_summary = _trajectory_summary(planned_trajectory, metrics)
    trajectory_source = "planned_trajectory" if planned_trajectory else "路径/时间序列估算"
    planning_events = _planning_events(payload)
    obstacle_model = payload.get("obstacle_model") or {}
    missing_notes = []
    if not payload.get("planned_path"):
        missing_notes.append("未提供 planned_path，路线图仅显示可用轨迹。")
    if not payload.get("executed_path"):
        missing_notes.append("未提供 executed_path，无法显示实际执行路径。")
    if not planned_trajectory:
        missing_notes.append("未提供 planned_trajectory，轨迹指标已从路径/时间序列估算。")
    if not planning_events:
        missing_notes.append("未提供 planning_events，规划器耗时仅显示为空图。")

    lines = [
        f"# {title}",
        "",
        "## 预设场景",
        "",
        "| 项目 | 值 |",
        "|---|---:|",
        f"| 预设名称 | {payload.get('preset', '')} |",
        f"| 运行引擎 | {payload.get('runtime_engine', '')} |",
        f"| 仿真耗时 s | {_fmt(payload.get('runtime_s'))} |",
        f"| 无人机从机数 | {_fmt(cfg.get('num_followers'))} |",
        f"| 编队类型 | {cfg.get('initial_formation', '')} |",
        f"| 航点数量 | {len(payload.get('task_waypoints') or payload.get('waypoints') or cfg.get('waypoints') or [])} |",
        f"| 传感器 | {_yes_no(cfg.get('sensor_enabled'))} |",
        f"| GNN danger 模式 | {_yes_no(cfg.get('danger_mode_enabled'))} |",
        f"| APF 编队避障 | {_apf_label(cfg)} |",
        f"| 轨迹优化 | {_yes_no(cfg.get('trajectory_optimizer_enabled'))} {cfg.get('trajectory_optimizer_method', '')} |",
        "",
        "## 路径规划路线参数",
        "",
        "| 参数 | 值 |",
        "|---|---:|",
        f"| 规划器 | {cfg.get('planner_kind', '')} |",
        f"| 规划模式 | {cfg.get('planner_mode', '')} |",
        f"| 栅格分辨率 m | {_fmt(cfg.get('planner_resolution'))} |",
        f"| 重规划周期 s | {_fmt(cfg.get('planner_replan_interval'))} |",
        f"| 规划前瞻 m | {_fmt(cfg.get('planner_horizon'))} |",
        f"| 规划路径长度 m | {_fmt(metrics.get('planned_path_length'))} |",
        f"| 执行路径长度 m | {_fmt(metrics.get('executed_path_length'))} |",
        f"| 轨迹指标来源 | {trajectory_source} |",
        f"| 轨迹路径长度 m | {_fmt(trajectory_summary.get('path_length'))} |",
        f"| 最大速度 m/s | {_fmt(trajectory_summary.get('max_speed'))} |",
        f"| 最大加速度 m/s² | {_fmt(trajectory_summary.get('max_acceleration'))} |",
        f"| jerk 平方积分 | {_fmt(trajectory_summary.get('jerk_squared_integral'))} |",
        f"| snap 平方积分 | {_fmt(trajectory_summary.get('snap_squared_integral'))} |",
    ]
    lines.extend(_map_knowledge_markdown(payload))
    if obstacle_model:
        lines.extend(_obstacle_model_markdown(obstacle_model))
    lines.extend(_formation_adaptation_markdown(payload, metrics, cfg))
    lines.extend([
        "",
        "## 规划器与规划耗时",
        "",
        "### 规划事件汇总",
        "",
        "| 阶段 | 规划器 | 事件数 | 接受数 | 拒绝数 | 平均耗时 s | 最大耗时 s | 平均路径点数 |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ])
    if planning_events:
        lines.extend(_planning_summary_rows(planning_events))
    else:
        lines.append("| 未提供 | 未提供 | 0 | 0 | 0 |  |  |  |")

    selected_events, omitted_count = _compact_planning_events(planning_events)
    if omitted_count > 0:
        lines.extend([
            "",
            f"> 规划事件明细已压缩：仅显示代表性 {len(selected_events)} 条，完整 {len(planning_events)} 条保留在 sim_result.json。",
        ])

    lines.extend([
        "",
        "### 规划事件明细",
        "",
        "| 阶段 | 规划器 | 仿真时间 s | 墙钟耗时 s | 路径点数 | 是否接受 | 回退原因 |",
        "|---|---|---:|---:|---:|---|---|",
    ])
    if selected_events:
        for event in selected_events:
            lines.append(
                "| "
                + " | ".join(
                    [
                        str(event.get("phase", "")),
                        str(event.get("planner", "")),
                        _fmt(event.get("t")),
                        _fmt(event.get("wall_time_s")),
                        _fmt(event.get("point_count")),
                        _yes_no(event.get("accepted")),
                        str(event.get("fallback_reason") or ""),
                    ]
                )
                + " |"
            )
    else:
        lines.append("| 未提供 | 未提供 |  |  |  |  |  |")

    lines.extend([
        "",
        "## 飞行参数",
        "",
        "| 参数 | 值 |",
        "|---|---:|",
        f"| 完成航点数 | {_fmt(payload.get('completed_waypoint_count'))} |",
        f"| 平均误差 m | {_fmt(metrics.get('mean_error_overall'))} |",
        f"| 最大误差 m | {_fmt(metrics.get('max_error_overall'))} |",
        f"| 最终误差 m | {_fmt(metrics.get('final_error_overall'))} |",
        f"| 硬碰撞区间数 | {_fmt(metrics.get('hard_collision_count'))} |",
        f"| 硬碰撞逐步记录数 | {_fmt(metrics.get('hard_collision_step_count'))} |",
        f"| 安全裕度告警区间数 | {_fmt(metrics.get('clearance_warning_count'))} |",
        f"| 安全裕度告警逐步记录数 | {_fmt(metrics.get('clearance_warning_step_count'))} |",
        f"| 最小障碍物有符号距离 m | {_fmt(metrics.get('min_obstacle_signed_distance'))} |",
        f"| 重规划次数 | {_fmt(metrics.get('replan_count'))} |",
        f"| 故障事件数 | {_fmt(metrics.get('fault_count'))} |",
        f"| 最小机间距 m | {_fmt(metrics.get('min_inter_drone_distance'))} |",
        f"| Downwash 命中次数 | {_fmt(metrics.get('downwash_hits'))} |",
        "",
        "## 图片结果",
        "",
        f"![场景与路线]({figure_dir_name}/{FIGURE_NAMES['route']})",
        "",
        f"![三维飞行轨迹]({figure_dir_name}/{FIGURE_NAMES['route_3d']})",
        "",
        f"![飞行参数]({figure_dir_name}/{FIGURE_NAMES['flight']})",
        "",
        f"![误差统计]({figure_dir_name}/{FIGURE_NAMES['errors']})",
        "",
        f"![飞行事件时间线]({figure_dir_name}/{FIGURE_NAMES['events']})",
        "",
        f"![规划器耗时]({figure_dir_name}/{FIGURE_NAMES['planning']})",
        "",
    ])
    if missing_notes:
        lines.extend(["## 数据缺失说明", ""])
        lines.extend(f"- {note}" for note in missing_notes)
        lines.append("")
    return "\n".join(lines)


def _default_report_title(payload: Mapping[str, Any]) -> str:
    preset = str(payload.get("preset") or "仿真")
    label = PRESET_LABELS.get(preset, preset)
    return f"{label} 仿真结果报告"


def _map_knowledge_markdown(payload: Mapping[str, Any]) -> list[str]:
    knowledge = payload.get("map_knowledge") or {}
    if not isinstance(knowledge, Mapping) or not knowledge:
        return []
    return [
        "",
        "## 地图知识状态",
        "",
        "| 项目 | 值 |",
        "|---|---:|",
        f"| 初始规划地图未知 | {_yes_no(knowledge.get('initial_map_unknown'))} |",
        f"| 真实障碍物数量 | {_fmt(knowledge.get('truth_obstacle_count'))} |",
        f"| 初始静态占据栅格数 | {_fmt(knowledge.get('planner_static_occupied_count'))} |",
        f"| 传感器发现占据栅格数 | {_fmt(knowledge.get('planner_sensor_occupied_count'))} |",
    ]


def _plot_route(payload: Mapping[str, Any], path: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 5))
    plotted = False
    plotted |= _plot_obstacles_2d(ax, payload.get("obstacle_model") or {})
    plotted |= _plot_xy(ax, _points(payload.get("task_waypoints") or payload.get("waypoints")), "任务航点", "o")
    plotted |= _plot_xy(ax, _points(payload.get("planned_path")), "规划路径", "-")
    plotted |= _plot_xy(ax, _points(payload.get("replanned_waypoints")), "重规划路径", "--")
    plotted |= _plot_xy(ax, _points(payload.get("executed_path")), "执行路径", "-")
    _scatter_event_points(ax, payload)
    _finish_axes(ax, "场景与路线", "X / m", "Y / m", plotted)
    return _save(fig, path)


def _plot_route_3d(payload: Mapping[str, Any], path: Path) -> Path:
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    plotted = _plot_obstacles_3d(ax, payload.get("obstacle_model") or {})
    for key, label in (("planned_path", "规划路径"), ("executed_path", "执行路径")):
        pts = _points(payload.get(key))
        if len(pts) > 0:
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], label=label)
            plotted = True
    waypoints = _points(payload.get("task_waypoints") or payload.get("waypoints"))
    if len(waypoints) > 0:
        ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], label="任务航点")
        plotted = True
    if not plotted:
        ax.text2D(0.2, 0.5, "未提供路线数据", transform=ax.transAxes)
    ax.set_title("三维飞行轨迹")
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Z / m")
    if plotted:
        ax.legend()
    return _save(fig, path)


def _plot_flight_parameters(payload: Mapping[str, Any], metrics: Mapping[str, Any], path: Path) -> Path:
    planned_trajectory = _trajectory_summary(payload.get("planned_trajectory") or {}, metrics)
    values = {
        "运行耗时": payload.get("runtime_s"),
        "完成航点": payload.get("completed_waypoint_count"),
        "最大速度": planned_trajectory.get("max_speed"),
        "最大加速度": planned_trajectory.get("max_acceleration"),
        "snap积分": planned_trajectory.get("snap_squared_integral"),
        "最小机间距": metrics.get("min_inter_drone_distance"),
    }
    return _plot_bar(values, "飞行参数", path)


def _plot_tracking_errors(payload: Mapping[str, Any], path: Path) -> Path:
    raw = payload.get("metrics") or {}
    labels = [f"从机{i + 1}" for i in range(max(len(raw.get("mean", [])), len(raw.get("max", [])), len(raw.get("final", []))))]
    fig, ax = plt.subplots(figsize=(7, 4))
    if labels:
        x = np.arange(len(labels))
        width = 0.25
        ax.bar(x - width, _pad(raw.get("mean", []), len(labels)), width, label="平均误差")
        ax.bar(x, _pad(raw.get("max", []), len(labels)), width, label="最大误差")
        ax.bar(x + width, _pad(raw.get("final", []), len(labels)), width, label="最终误差")
        ax.set_xticks(x, labels)
        ax.legend()
    else:
        ax.text(0.5, 0.5, "未提供误差统计", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("误差统计")
    ax.set_ylabel("误差 / m")
    return _save(fig, path)


def _plot_event_timeline(payload: Mapping[str, Any], path: Path) -> Path:
    events: list[tuple[float, str]] = []
    for key, label in (
        ("waypoint_events", "航点到达"),
        ("replan_events", "重规划"),
        ("collision_log", "碰撞"),
        ("fault_log", "故障"),
    ):
        for event in payload.get(key) or []:
            t = _as_float(event.get("t"))
            if t is not None:
                events.append((t, label))
    fig, ax = plt.subplots(figsize=(7, 3.8))
    if events:
        label_order = ["航点到达", "重规划", "碰撞", "故障"]
        y_map = {label: idx for idx, label in enumerate(label_order)}
        for t, label in events:
            ax.scatter(t, y_map[label], label=label)
        ax.set_yticks(range(len(label_order)), label_order)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), loc="upper right")
    else:
        ax.text(0.5, 0.5, "未提供带时间戳的飞行事件", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("飞行事件时间线")
    ax.set_xlabel("仿真时间 / s")
    return _save(fig, path)


def _plot_planning_times(payload: Mapping[str, Any], path: Path) -> Path:
    events = _planning_events(payload)
    fig, ax = plt.subplots(figsize=(7, 4))
    points: dict[str, list[tuple[float, float, bool | None]]] = {}
    for idx, event in enumerate(events):
        t = _as_float(event.get("t"))
        wall_time = _as_float(event.get("wall_time_s"))
        if wall_time is None:
            continue
        if t is None:
            t = float(idx)
        key = f"{event.get('planner', '')}/{event.get('phase', '')}"
        points.setdefault(key, []).append((t, wall_time, event.get("accepted")))

    if points:
        for label, values in points.items():
            xs = [item[0] for item in values]
            ys = [item[1] for item in values]
            colors = ["tab:blue" if item[2] is not False else "tab:red" for item in values]
            ax.scatter(xs, ys, s=16, alpha=0.75, label=label, c=colors)
        ax.set_xlabel("仿真时间 / s")
        ax.set_ylabel("墙钟耗时 / s")
        ax.legend(loc="best", fontsize=8)
    else:
        ax.text(0.5, 0.5, "未提供数据", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("规划器耗时")
    return _save(fig, path)


def _plot_bar(values: Mapping[str, Any], title: str, path: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 4))
    numeric = {key: _as_float(value) for key, value in values.items()}
    numeric = {key: value for key, value in numeric.items() if value is not None}
    if numeric:
        labels = list(numeric.keys())
        ax.bar(labels, list(numeric.values()))
        ax.tick_params(axis="x", rotation=25)
    else:
        ax.text(0.5, 0.5, "未提供数据", ha="center", va="center", transform=ax.transAxes)
    ax.set_title(title)
    return _save(fig, path)


def _trajectory_summary(planned_trajectory: Mapping[str, Any], metrics: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "path_length": _first_value(
            planned_trajectory.get("path_length"),
            metrics.get("trajectory_path_length"),
            metrics.get("planned_path_length"),
        ),
        "max_speed": _first_value(
            planned_trajectory.get("max_speed"),
            metrics.get("trajectory_max_speed"),
        ),
        "max_acceleration": _first_value(
            planned_trajectory.get("max_acceleration"),
            metrics.get("trajectory_max_acceleration"),
        ),
        "jerk_squared_integral": _first_value(
            planned_trajectory.get("jerk_squared_integral"),
            metrics.get("trajectory_jerk_squared_integral"),
        ),
        "snap_squared_integral": _first_value(
            planned_trajectory.get("snap_squared_integral"),
            metrics.get("trajectory_snap_squared_integral"),
        ),
    }


def _planning_summary_rows(events: Sequence[Mapping[str, Any]]) -> list[str]:
    groups: dict[tuple[str, str], list[Mapping[str, Any]]] = {}
    for event in events:
        key = (str(event.get("phase", "")), str(event.get("planner", "")))
        groups.setdefault(key, []).append(event)

    rows = []
    for (phase, planner), items in sorted(groups.items()):
        accepted = sum(1 for item in items if item.get("accepted") is True)
        rejected = sum(1 for item in items if item.get("accepted") is False)
        wall_times = [_as_float(item.get("wall_time_s")) for item in items]
        wall_times = [value for value in wall_times if value is not None]
        point_counts = [_as_float(item.get("point_count")) for item in items]
        point_counts = [value for value in point_counts if value is not None]
        mean_wall = sum(wall_times) / len(wall_times) if wall_times else None
        max_wall = max(wall_times) if wall_times else None
        mean_points = sum(point_counts) / len(point_counts) if point_counts else None
        rows.append(
            "| "
            + " | ".join([
                phase,
                planner,
                _fmt(len(items)),
                _fmt(accepted),
                _fmt(rejected),
                _fmt(mean_wall),
                _fmt(max_wall),
                _fmt(mean_points),
            ])
            + " |"
        )
    return rows


def _compact_planning_events(
    events: Sequence[Mapping[str, Any]],
    *,
    max_rows: int = 24,
) -> tuple[list[Mapping[str, Any]], int]:
    event_list = list(events)
    if len(event_list) <= max_rows:
        return event_list, 0

    selected_indexes: set[int] = set(range(min(8, len(event_list))))
    selected_indexes.update(range(max(len(event_list) - 8, 0), len(event_list)))

    accepted = [idx for idx, event in enumerate(event_list) if event.get("accepted") is True]
    rejected = [idx for idx, event in enumerate(event_list) if event.get("accepted") is False]
    selected_indexes.update(accepted[:4])
    selected_indexes.update(rejected[:4])

    wall_candidates = [
        (idx, _as_float(event.get("wall_time_s")))
        for idx, event in enumerate(event_list)
    ]
    wall_candidates = [(idx, value) for idx, value in wall_candidates if value is not None]
    wall_candidates.sort(key=lambda item: item[1], reverse=True)
    selected_indexes.update(idx for idx, _ in wall_candidates[:4])

    ordered = sorted(selected_indexes)
    if len(ordered) > max_rows:
        keep_front = max_rows // 2
        keep_back = max_rows - keep_front
        ordered = ordered[:keep_front] + ordered[-keep_back:]
    selected = [event_list[idx] for idx in ordered]
    return selected, len(event_list) - len(selected)


def _obstacle_model_markdown(obstacle_model: Mapping[str, Any]) -> list[str]:
    primitives = list(obstacle_model.get("primitives") or [])
    counts: dict[str, int] = {}
    for primitive in primitives:
        ptype = str(primitive.get("type", "unknown"))
        counts[ptype] = counts.get(ptype, 0) + 1

    lines = ["", "## 障碍物建模", ""]
    bounds = obstacle_model.get("bounds") or []
    if bounds:
        lines.extend([
            "| 地图边界 | 值 |",
            "|---|---|",
            f"| min | {_format_vec(bounds[0]) if len(bounds) > 0 else ''} |",
            f"| max | {_format_vec(bounds[1]) if len(bounds) > 1 else ''} |",
            "",
        ])

    lines.extend([
        "| 类型 | 数量 |",
        "|---|---:|",
    ])
    if counts:
        for ptype in sorted(counts):
            lines.append(f"| {ptype} | {counts[ptype]} |")
    else:
        lines.append("| 未提供 | 0 |")

    if primitives:
        lines.extend(["", "| ID | 类型 | 主要参数 |", "|---|---|---|"])
        for idx, primitive in enumerate(primitives[:20]):
            lines.append(
                f"| {primitive.get('id', idx)} | {primitive.get('type', '')} | {_primitive_summary(primitive)} |"
            )
        if len(primitives) > 20:
            lines.append(f"| ... | ... | 其余 {len(primitives) - 20} 个障碍物见 sim_result.json |")
    return lines


def _formation_adaptation_markdown(
    payload: Mapping[str, Any],
    metrics: Mapping[str, Any],
    cfg: Mapping[str, Any],
) -> list[str]:
    events = [event for event in (payload.get("formation_adaptation_events") or []) if isinstance(event, Mapping)]
    enabled = bool(cfg.get("formation_adaptation_enabled"))
    lookahead_enabled = bool(cfg.get("formation_lookahead_enabled"))
    rrt_enabled = bool(cfg.get("formation_lookahead_rrt_enabled"))
    if not (events or enabled or lookahead_enabled or rrt_enabled):
        return []

    lines = [
        "",
        "## 编队调控机制",
        "",
        "| 项目 | 值 |",
        "|---|---:|",
        f"| 队形自适应 | {_yes_no(enabled)} |",
        f"| 前瞻窗口 | {_yes_no(lookahead_enabled)} |",
        f"| 前瞻 RRT escape | {_yes_no(rrt_enabled)} |",
        f"| 队形调整事件数 | {_fmt(metrics.get('formation_adaptation_count'))} |",
        f"| 最终队形 | {metrics.get('formation_adaptation_last') or ''} |",
        f"| 最后调整原因 | {metrics.get('formation_adaptation_last_reason') or ''} |",
        f"| 前瞻阻断次数 | {_fmt(metrics.get('lookahead_reference_blocked_count'))} |",
        f"| RRT escape 尝试数 | {_fmt(metrics.get('rrt_escape_attempt_count'))} |",
        f"| RRT escape 接受数 | {_fmt(metrics.get('rrt_escape_accepted_count'))} |",
        f"| RRT escape 失败数 | {_fmt(metrics.get('rrt_escape_failed_count'))} |",
    ]
    if events:
        lines.extend([
            "",
            "### 编队调控事件",
            "",
            "| 时间 s | 类型 | from | to | 原因 | 规划器 | 点数 | 裕度/转角 |",
            "|---|---|---|---|---|---|---:|---|",
        ])
        for event in events[:20]:
            kind = str(event.get("kind") or "formation_switch")
            detail = []
            if event.get("clearance_margin") is not None:
                detail.append(f"clearance_margin={_fmt(event.get('clearance_margin'))}")
            if event.get("max_turn_angle_rad") is not None:
                detail.append(f"max_turn={_fmt(event.get('max_turn_angle_rad'))}")
            lines.append(
                "| "
                + " | ".join(
                    [
                        _fmt(event.get("t")),
                        kind,
                        str(event.get("from") or ""),
                        str(event.get("to") or ""),
                        str(event.get("reason") or ""),
                        str(event.get("planner") or ""),
                        _fmt(event.get("point_count")),
                        "; ".join(detail),
                    ]
                )
                + " |"
            )
        if len(events) > 20:
            lines.append(f"| ... | ... | ... | ... | ... | ... |  | 其余 {len(events) - 20} 条事件见 sim_result.json |")
    return lines


def _plot_obstacles_2d(ax, obstacle_model: Mapping[str, Any]) -> bool:
    plotted = False
    bounds = obstacle_model.get("bounds") or []
    if len(bounds) >= 2:
        mn = _vec(bounds[0])
        mx = _vec(bounds[1])
        if mn is not None and mx is not None:
            rect = patches.Rectangle(
                (mn[0], mn[1]),
                max(mx[0] - mn[0], 0.0),
                max(mx[1] - mn[1], 0.0),
                fill=False,
                linestyle=":",
                linewidth=1.0,
                edgecolor="0.45",
                label="地图边界",
            )
            ax.add_patch(rect)
            plotted = True

    obstacle_label = True
    for primitive in obstacle_model.get("primitives") or []:
        ptype = primitive.get("type")
        label = "障碍物" if obstacle_label else None
        if ptype == "aabb":
            mn = _vec(primitive.get("min"))
            mx = _vec(primitive.get("max"))
            if mn is None or mx is None:
                continue
            rect = patches.Rectangle(
                (mn[0], mn[1]),
                max(mx[0] - mn[0], 0.0),
                max(mx[1] - mn[1], 0.0),
                facecolor="tab:red",
                edgecolor="tab:red",
                alpha=0.18,
                label=label,
            )
            ax.add_patch(rect)
            plotted = True
            obstacle_label = False
        elif ptype in {"sphere", "cylinder"}:
            center = _vec(primitive.get("center"))
            radius = _as_float(primitive.get("radius"))
            if center is None or radius is None:
                continue
            circle = patches.Circle(
                (center[0], center[1]),
                radius,
                facecolor="tab:red",
                edgecolor="tab:red",
                alpha=0.18,
                label=label,
            )
            ax.add_patch(circle)
            plotted = True
            obstacle_label = False
    ax.autoscale_view()
    return plotted


def _plot_obstacles_3d(ax, obstacle_model: Mapping[str, Any]) -> bool:
    plotted = False
    for primitive in obstacle_model.get("primitives") or []:
        ptype = primitive.get("type")
        if ptype == "aabb":
            mn = _vec(primitive.get("min"))
            mx = _vec(primitive.get("max"))
            if mn is None or mx is None:
                continue
            _plot_aabb_edges(ax, mn, mx)
            plotted = True
        elif ptype == "sphere":
            center = _vec(primitive.get("center"))
            radius = _as_float(primitive.get("radius"))
            if center is None or radius is None:
                continue
            u = np.linspace(0, 2 * np.pi, 18)
            v = np.linspace(0, np.pi, 9)
            x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
            y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
            z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_wireframe(x, y, z, color="tab:red", linewidth=0.35, alpha=0.35)
            plotted = True
        elif ptype == "cylinder":
            center = _vec(primitive.get("center"))
            radius = _as_float(primitive.get("radius"))
            z_min = _as_float(primitive.get("z_min"))
            z_max = _as_float(primitive.get("z_max"))
            if center is None or radius is None or z_min is None or z_max is None:
                continue
            theta = np.linspace(0, 2 * np.pi, 18)
            z = np.linspace(z_min, z_max, 5)
            theta_grid, z_grid = np.meshgrid(theta, z)
            x = center[0] + radius * np.cos(theta_grid)
            y = center[1] + radius * np.sin(theta_grid)
            ax.plot_wireframe(x, y, z_grid, color="tab:red", linewidth=0.35, alpha=0.35)
            plotted = True
    return plotted


def _plot_aabb_edges(ax, mn: np.ndarray, mx: np.ndarray) -> None:
    corners = np.array([
        [mn[0], mn[1], mn[2]],
        [mx[0], mn[1], mn[2]],
        [mx[0], mx[1], mn[2]],
        [mn[0], mx[1], mn[2]],
        [mn[0], mn[1], mx[2]],
        [mx[0], mn[1], mx[2]],
        [mx[0], mx[1], mx[2]],
        [mn[0], mx[1], mx[2]],
    ])
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color="tab:red",
            linewidth=0.5,
            alpha=0.5,
        )


def _planning_events(payload: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    events = payload.get("planning_events") or []
    if events:
        return list(events)
    timing = payload.get("timing") or {}
    planning_s = timing.get("planning_s")
    if planning_s is not None:
        return [{
            "t": 0.0,
            "phase": "cpp_planning",
            "planner": "cpp",
            "wall_time_s": planning_s,
            "point_count": len(payload.get("replanned_waypoints") or payload.get("executed_path") or []),
            "accepted": True,
            "fallback_reason": "",
        }]
    return []


def _plot_xy(ax, points: np.ndarray, label: str, style: str) -> bool:
    if len(points) == 0:
        return False
    if style == "o":
        ax.scatter(points[:, 0], points[:, 1], label=label)
    else:
        ax.plot(points[:, 0], points[:, 1], style, label=label)
    return True


def _scatter_event_points(ax, payload: Mapping[str, Any]) -> None:
    for event in payload.get("collision_log") or []:
        point = _event_point(event)
        if point is not None:
            ax.scatter([point[0]], [point[1]], marker="x", s=80, label="碰撞")


def _event_point(event: Mapping[str, Any]) -> np.ndarray | None:
    for key in ("position", "pos", "point"):
        value = event.get(key)
        pts = _points([value] if value is not None else None)
        if len(pts) == 1:
            return pts[0]
    return None


def _finish_axes(ax, title: str, xlabel: str, ylabel: str, plotted: bool) -> None:
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.axis("equal")
    if plotted:
        ax.legend()
    else:
        ax.text(0.5, 0.5, "未提供路线数据", ha="center", va="center", transform=ax.transAxes)


def _save(fig, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Tight layout not applied.*", category=UserWarning)
        fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _points(value: Any) -> np.ndarray:
    if value is None:
        return np.zeros((0, 3), dtype=float)
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return np.zeros((0, 3), dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2:
        return np.zeros((0, 3), dtype=float)
    if arr.shape[1] == 2:
        arr = np.column_stack([arr, np.zeros(len(arr))])
    return arr[:, :3]


def _vec(value: Any) -> np.ndarray | None:
    pts = _points([value] if value is not None else None)
    if len(pts) != 1:
        return None
    return pts[0]


def _format_vec(value: Any) -> str:
    vec = _vec(value)
    if vec is None:
        return ""
    return f"({_fmt(vec[0])}, {_fmt(vec[1])}, {_fmt(vec[2])})"


def _primitive_summary(primitive: Mapping[str, Any]) -> str:
    ptype = primitive.get("type")
    if ptype == "aabb":
        return f"min={_format_vec(primitive.get('min'))}; max={_format_vec(primitive.get('max'))}"
    if ptype == "sphere":
        return f"center={_format_vec(primitive.get('center'))}; r={_fmt(primitive.get('radius'))}"
    if ptype == "cylinder":
        return (
            f"center={_format_vec(primitive.get('center'))}; "
            f"r={_fmt(primitive.get('radius'))}; "
            f"z=[{_fmt(primitive.get('z_min'))}, {_fmt(primitive.get('z_max'))}]"
        )
    return ""


def _pad(values: Sequence[Any], length: int) -> list[float]:
    out = []
    for value in list(values)[:length]:
        out.append(float(value))
    while len(out) < length:
        out.append(0.0)
    return out


def _fmt(value: Any) -> str:
    number = _as_float(value)
    if number is None:
        return ""
    if float(number).is_integer():
        return str(int(number))
    return f"{number:.6g}"


def _as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _first_value(*values: Any) -> Any:
    for value in values:
        if _as_float(value) is not None:
            return value
    return None


def _yes_no(value: Any) -> str:
    if value is True:
        return "是"
    if value is False:
        return "否"
    return ""


def _apf_label(cfg: dict[str, Any]) -> str:
    profile = str(cfg.get("apf_paper1_profile") or "off")
    if profile == "off":
        return "否"
    return f"是 ({profile})"


__all__ = ["generate_result_report"]
