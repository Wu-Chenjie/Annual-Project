from __future__ import annotations

import numpy as np


def _collision_interval_count(events: list[dict], dt: float) -> int:
    """将连续的碰撞事件归并为碰撞区间，间隔 > dt*1.5 视为新区间。"""
    if not events:
        return 0
    sorted_events = sorted(events, key=lambda e: (e["drone"], e["t"]))
    intervals = 0
    current_drone = ""
    previous_t = -1.0
    max_gap = max(dt * 1.5, 1e-6)
    for event in sorted_events:
        if event["drone"] != current_drone or previous_t < 0.0 or event["t"] - previous_t > max_gap:
            intervals += 1
            current_drone = event["drone"]
        previous_t = event["t"]
    return intervals


def _hard_collision_step_count(events: list[dict], obstacles) -> int:
    """统计 signed_distance < 0 的硬碰撞步数（机身进入障碍物内部）。"""
    count = 0
    for event in events:
        pos = np.array(event["pos"], dtype=float)
        if obstacles.signed_distance(pos) < 0.0:
            count += 1
    return count


def _hard_collision_interval_count(events: list[dict], obstacles, dt: float) -> int:
    """硬碰撞事件的区间计数。"""
    hard = [e for e in events if obstacles.signed_distance(np.array(e["pos"], dtype=float)) < 0.0]
    return _collision_interval_count(hard, dt)


def _min_airframe_signed_distance(
    leader_traj: np.ndarray,
    follower_trajs: list[np.ndarray],
    obstacles,
) -> float:
    """所有无人机航迹点中距障碍物的最小有符号距离。"""
    if not obstacles:
        return 0.0
    best = float("inf")
    for p in leader_traj:
        best = min(best, obstacles.signed_distance(p))
    for ft in follower_trajs:
        for p in ft:
            best = min(best, obstacles.signed_distance(p))
    return float(best) if np.isfinite(best) else 0.0
