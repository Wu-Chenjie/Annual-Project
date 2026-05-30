"""Preset registry and public config helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

try:
    from simulations.formation_simulation import SimulationConfig
except ModuleNotFoundError:
    from ..simulations.formation_simulation import SimulationConfig

PKG = Path(__file__).resolve().parent.parent
PRESET_METADATA_PATH = PKG / "preset_metadata.json"

from .basic import _config_basic, _config_custom, _config_obstacle
from .fault_tolerance import (
    _config_fault_tolerance,
    _config_fault_tolerance_online,
    _config_fault_tolerance_online_unknown,
    _config_fault_tolerance_unknown,
)
from .stress_tests import (
    _config_formation_maze_stress_online,
    _config_formation_maze_stress_online_unknown,
    _config_rrt_dual_channel_online,
    _config_rrt_dual_channel_online_unknown,
)
from .unknown_map import (
    _config_company_cubicles_online_unknown,
    _config_company_cubicles_unknown,
    _config_laboratory_online_unknown,
    _config_laboratory_unknown,
    _config_meeting_room_online_unknown,
    _config_meeting_room_unknown,
    _config_obstacle_unknown,
    _config_school_corridor_online_unknown,
    _config_school_corridor_unknown,
    _config_unknown_map_online,
    _config_warehouse_a_unknown,
    _config_warehouse_danger_unknown,
    _config_warehouse_online_unknown,
    _config_warehouse_unknown,
)
from .warehouse import (
    _config_company_cubicles,
    _config_company_cubicles_online,
    _config_laboratory,
    _config_laboratory_online,
    _config_meeting_room,
    _config_meeting_room_online,
    _config_school_corridor,
    _config_school_corridor_online,
    _config_warehouse,
    _config_warehouse_a,
    _config_warehouse_danger,
    _config_warehouse_online,
)


AVAILABLE_PRESETS = [
    "basic",
    "obstacle",
    "obstacle_unknown",
    "warehouse",
    "warehouse_a",
    "warehouse_online",
    "warehouse_danger",
    "warehouse_unknown",
    "warehouse_a_unknown",
    "warehouse_online_unknown",
    "warehouse_danger_unknown",
    "fault_tolerance",
    "fault_tolerance_online",
    "fault_tolerance_unknown",
    "fault_tolerance_online_unknown",
    "school_corridor",
    "school_corridor_online",
    "school_corridor_unknown",
    "school_corridor_online_unknown",
    "company_cubicles",
    "company_cubicles_online",
    "company_cubicles_unknown",
    "company_cubicles_online_unknown",
    "meeting_room",
    "meeting_room_online",
    "meeting_room_unknown",
    "meeting_room_online_unknown",
    "rrt_dual_channel_online",
    "rrt_dual_channel_online_unknown",
    "formation_maze_stress_online",
    "formation_maze_stress_online_unknown",
    "unknown_map_online",
    "laboratory",
    "laboratory_online",
    "laboratory_unknown",
    "laboratory_online_unknown",
    "custom",
]



def load_preset_metadata() -> dict[str, dict[str, str]]:
    """Load machine-readable preset metadata shared by CLI docs and Web."""
    raw = json.loads(PRESET_METADATA_PATH.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("preset metadata must be a JSON object")
    metadata: dict[str, dict[str, str]] = {}
    for preset, fields in raw.items():
        if not isinstance(fields, dict):
            raise ValueError(f"preset metadata entry {preset!r} must be an object")
        metadata[str(preset)] = {str(key): str(value) for key, value in fields.items()}
    return metadata



_PRESETS: dict[str, Callable[[], SimulationConfig]] = {
    "basic": _config_basic,
    "obstacle": _config_obstacle,
    "obstacle_unknown": _config_obstacle_unknown,
    "warehouse": _config_warehouse,
    "warehouse_a": _config_warehouse_a,
    "warehouse_online": _config_warehouse_online,
    "warehouse_danger": _config_warehouse_danger,
    "warehouse_unknown": _config_warehouse_unknown,
    "warehouse_a_unknown": _config_warehouse_a_unknown,
    "warehouse_online_unknown": _config_warehouse_online_unknown,
    "warehouse_danger_unknown": _config_warehouse_danger_unknown,
    "fault_tolerance": _config_fault_tolerance,
    "fault_tolerance_online": _config_fault_tolerance_online,
    "fault_tolerance_unknown": _config_fault_tolerance_unknown,
    "fault_tolerance_online_unknown": _config_fault_tolerance_online_unknown,
    "school_corridor": _config_school_corridor,
    "school_corridor_online": _config_school_corridor_online,
    "school_corridor_unknown": _config_school_corridor_unknown,
    "school_corridor_online_unknown": _config_school_corridor_online_unknown,
    "company_cubicles": _config_company_cubicles,
    "company_cubicles_online": _config_company_cubicles_online,
    "company_cubicles_unknown": _config_company_cubicles_unknown,
    "company_cubicles_online_unknown": _config_company_cubicles_online_unknown,
    "meeting_room": _config_meeting_room,
    "meeting_room_online": _config_meeting_room_online,
    "meeting_room_unknown": _config_meeting_room_unknown,
    "meeting_room_online_unknown": _config_meeting_room_online_unknown,
    "rrt_dual_channel_online": _config_rrt_dual_channel_online,
    "rrt_dual_channel_online_unknown": _config_rrt_dual_channel_online_unknown,
    "formation_maze_stress_online": _config_formation_maze_stress_online,
    "formation_maze_stress_online_unknown": _config_formation_maze_stress_online_unknown,
    "unknown_map_online": _config_unknown_map_online,
    "laboratory": _config_laboratory,
    "laboratory_online": _config_laboratory_online,
    "laboratory_unknown": _config_laboratory_unknown,
    "laboratory_online_unknown": _config_laboratory_online_unknown,
    "custom": _config_custom,
}



def get_config(preset: str = "basic") -> SimulationConfig:
    """返回指定预设场景的仿真配置。

    可用预设:
        basic                基础编队验证（30s，方形航线）
        obstacle             简单障碍物避障（三柱，厘米级精度）
        obstacle_unknown     简单障碍物完全未知版（初始地图为空 + 传感器探索）
        warehouse            工业仓库复杂场景（在线 A* + 传感器 + 队形切换）
        warehouse_a          仓库场景 A* 版（在线 A* + ESDF + Danger 模式）
        warehouse_online     仓库场景在线版（A* + 传感器 + D* Lite）
        warehouse_danger     仓库在线版 + GNN 双模式 + 改进 APF（保守档）
        warehouse_unknown    仓库完全未知版（初始地图为空 + 传感器逐步发现）
        warehouse_a_unknown  仓库 A* 完全未知版
        warehouse_online_unknown 仓库在线完全未知版
        warehouse_danger_unknown 仓库 Danger 完全未知版
        fault_tolerance      容错测试场景（故障注入+拓扑重构）
        fault_tolerance_online 容错在线版
        fault_tolerance_unknown 容错完全未知版
        fault_tolerance_online_unknown 容错在线完全未知版
        school_corridor      学校走廊场景（离线 A*）
        school_corridor_online 学校走廊在线版（A* + 传感器 + D* Lite）
        school_corridor_unknown 学校走廊完全未知版
        school_corridor_online_unknown 学校走廊在线完全未知版
        company_cubicles     公司格子间场景（离线 Hybrid A*）
        company_cubicles_online 公司格子间在线版（Hybrid A* + 传感器 + D* Lite）
        company_cubicles_unknown 公司格子间完全未知版
        company_cubicles_online_unknown 公司格子间在线完全未知版
        meeting_room         会议室场景（离线 A*，cm 级精度）
        meeting_room_online  会议室在线版（A* + 传感器 + 实时重规划）
        meeting_room_unknown 会议室完全未知版
        meeting_room_online_unknown 会议室在线完全未知版
        rrt_dual_channel_online RRT 双通道绕行压力测试（前瞻窗口 + RRT escape）
        rrt_dual_channel_online_unknown RRT 双通道完全未知版
        formation_maze_stress_online 编队迷宫压力测试（狭长通道 + 急转弯 + 队形切换）
        formation_maze_stress_online_unknown 编队迷宫完全未知版
        unknown_map_online  完全未知地图在线探索（初始规划地图为空，传感器发现障碍）
        laboratory           实验室场景（离线 A*）
        laboratory_online    实验室在线版（Hybrid A* + 传感器 + D* Lite）
        laboratory_unknown   实验室完全未知版
        laboratory_online_unknown 实验室在线完全未知版
        custom               自定义（修改此函数返回值即可）
    """
    builder = _PRESETS.get(preset)
    if builder is None:
        import difflib
        suggestions = difflib.get_close_matches(preset, AVAILABLE_PRESETS, n=3, cutoff=0.4)
        hint = f"。你是否想输入: {', '.join(suggestions)}？" if suggestions else ""
        raise ValueError(
            f"未知预设: '{preset}'，可用: {', '.join(AVAILABLE_PRESETS)}" + hint
        )
    return builder()

