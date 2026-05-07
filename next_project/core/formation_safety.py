"""编队安全辅助模块。

用途
----
将多机安全走廊中的局部安全逻辑从场景主循环中抽离，包括：
1) follower 目标点的安全收缩；
2) 最小机间距检查；
3) 简化下洗区判定。

原理
----
不直接改写全局规划器，而是在局部执行层对候选目标和机间相对几何做守门，
保证窄通道与近机交互时先满足安全约束，再尽量保留队形。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


ArrayLike = np.ndarray
SignedDistanceFn = Callable[[ArrayLike], float]
SegmentSafeFn = Callable[[ArrayLike, float], bool]
ProjectToFreeFn = Callable[..., ArrayLike]


@dataclass(frozen=True)
class DownwashZone:
    """下洗保护区参数。"""

    radius: float
    height: float


@dataclass(frozen=True)
class FormationSafetyConfig:
    """编队局部安全层配置。"""

    enabled: bool = False
    min_inter_drone_distance: float = 0.35
    downwash_radius: float = 0.45
    downwash_height: float = 0.80
    follower_shrink_steps: int = 10
    follower_clearance_bias: float = 0.05
    conflict_vertical_step: float = 0.35
    conflict_lateral_step: float = 0.30
    recovery_hold_steps: int = 4
    recovery_clearance_margin: float = 0.10


def min_inter_drone_distance(positions: Iterable[ArrayLike]) -> float:
    """返回所有无人机之间的最小欧氏距离。"""
    pts = [np.asarray(p, dtype=float) for p in positions]
    if len(pts) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            best = min(best, float(np.linalg.norm(pts[i] - pts[j])))
    return best


def downwash_zone(radius: float, height: float) -> DownwashZone:
    """构造简化下洗保护区。"""
    return DownwashZone(radius=float(radius), height=float(height))


def is_in_downwash_zone(
    upper: ArrayLike,
    lower: ArrayLike,
    zone: DownwashZone,
) -> bool:
    """判断 lower 是否落入 upper 的下洗保护区。"""
    upper = np.asarray(upper, dtype=float)
    lower = np.asarray(lower, dtype=float)
    dz = float(upper[2] - lower[2])
    if dz <= 0.0 or dz > zone.height:
        return False
    lateral = float(np.linalg.norm(upper[:2] - lower[:2]))
    return lateral <= zone.radius


def follower_safety_correction(
    leader_pos: ArrayLike,
    raw_target: ArrayLike,
    *,
    current_pos: ArrayLike | None,
    signed_distance: SignedDistanceFn,
    segment_is_safe: SegmentSafeFn,
    project_to_free: ProjectToFreeFn | None,
    min_clearance: float,
    shrink_steps: int = 10,
) -> np.ndarray:
    """将 follower 目标约束到安全且尽量保持队形的位置。"""
    leader_pos = np.asarray(leader_pos, dtype=float)
    raw_target = np.asarray(raw_target, dtype=float)
    current = None if current_pos is None else np.asarray(current_pos, dtype=float)

    candidates: list[np.ndarray] = []
    for scale in np.linspace(1.0, 0.0, max(2, int(shrink_steps)) + 1):
        candidates.append(leader_pos + (raw_target - leader_pos) * float(scale))

    if project_to_free is not None:
        try:
            projected = project_to_free(
                raw_target,
                prefer=leader_pos - raw_target,
                min_clearance=min_clearance,
                max_radius_m=max(2.0, float(np.linalg.norm(raw_target - leader_pos)) + 1.0),
            )
            candidates.insert(1, np.asarray(projected, dtype=float))
        except Exception:
            pass

    seen: set[tuple[float, float, float]] = set()
    for candidate in candidates:
        candidate = np.asarray(candidate, dtype=float)
        key = tuple(np.round(candidate, 4))
        if key in seen:
            continue
        seen.add(key)
        if float(signed_distance(candidate)) < min_clearance - 1e-8:
            continue
        if current is not None:
            segment = np.vstack([current, candidate])
            if not segment_is_safe(segment, min_clearance):
                continue
        return candidate.copy()

    if current is not None and float(signed_distance(current)) >= 0.0:
        return current.copy()
    return leader_pos.copy()


def deconflict_follower_target(
    candidate_target: ArrayLike,
    *,
    leader_pos: ArrayLike,
    reserved_positions: Iterable[ArrayLike],
    current_pos: ArrayLike | None,
    signed_distance: SignedDistanceFn,
    segment_is_safe: SegmentSafeFn,
    min_clearance: float,
    min_inter_distance: float,
    downwash: DownwashZone | None,
    vertical_step: float = 0.35,
    lateral_step: float = 0.30,
    preferred_target: ArrayLike | None = None,
) -> np.ndarray:
    """对 follower 候选目标做轻量去冲突修正。

    优先保持原目标；若与已保留目标过近或进入下洗区，则尝试
    侧向平移和小幅错层，选择第一个满足几何与路径安全条件的候选。
    """
    leader = np.asarray(leader_pos, dtype=float)
    candidate = np.asarray(candidate_target, dtype=float)
    current = None if current_pos is None else np.asarray(current_pos, dtype=float)
    reserved = [np.asarray(p, dtype=float) for p in reserved_positions]

    base_dir = candidate[:2] - leader[:2]
    base_norm = float(np.linalg.norm(base_dir))
    if base_norm > 1e-9:
        tangent_xy = np.array([-base_dir[1], base_dir[0]], dtype=float) / base_norm
    else:
        tangent_xy = np.array([0.0, 1.0], dtype=float)

    offsets = [
        np.zeros(3, dtype=float),
        np.array([tangent_xy[0] * lateral_step, tangent_xy[1] * lateral_step, 0.0], dtype=float),
        np.array([-tangent_xy[0] * lateral_step, -tangent_xy[1] * lateral_step, 0.0], dtype=float),
        np.array([0.0, 0.0, vertical_step], dtype=float),
        np.array([0.0, 0.0, -vertical_step], dtype=float),
        np.array([tangent_xy[0] * lateral_step, tangent_xy[1] * lateral_step, vertical_step], dtype=float),
        np.array([-tangent_xy[0] * lateral_step, -tangent_xy[1] * lateral_step, vertical_step], dtype=float),
        np.array([tangent_xy[0] * lateral_step, tangent_xy[1] * lateral_step, -vertical_step], dtype=float),
        np.array([-tangent_xy[0] * lateral_step, -tangent_xy[1] * lateral_step, -vertical_step], dtype=float),
    ]

    def is_safe_target(point: np.ndarray, clearance_margin: float = 0.0) -> bool:
        if float(signed_distance(point)) < min_clearance - 1e-8:
            return False
        if current is not None:
            if not segment_is_safe(np.vstack([current, point]), min_clearance):
                return False
        for other in reserved:
            if float(np.linalg.norm(point - other)) < (min_inter_distance + clearance_margin) - 1e-8:
                return False
            if downwash is not None:
                if is_in_downwash_zone(point, other, downwash) or is_in_downwash_zone(other, point, downwash):
                    return False
        return True

    if preferred_target is not None:
        preferred = np.asarray(preferred_target, dtype=float)
        if is_safe_target(preferred):
            return preferred.copy()

    for delta in offsets:
        probe = candidate + delta
        if is_safe_target(probe):
            return probe.copy()
    return candidate.copy()


def nominal_target_ready_for_recovery(
    nominal_target: ArrayLike,
    *,
    reserved_positions: Iterable[ArrayLike],
    current_pos: ArrayLike | None,
    signed_distance: SignedDistanceFn,
    segment_is_safe: SegmentSafeFn,
    min_clearance: float,
    min_inter_distance: float,
    downwash: DownwashZone | None,
    recovery_margin: float = 0.10,
) -> bool:
    """判断名义目标是否已连续满足恢复条件。"""
    point = np.asarray(nominal_target, dtype=float)
    if float(signed_distance(point)) < (min_clearance + recovery_margin) - 1e-8:
        return False
    if current_pos is not None:
        current = np.asarray(current_pos, dtype=float)
        if not segment_is_safe(np.vstack([current, point]), min_clearance):
            return False
    for other in reserved_positions:
        other = np.asarray(other, dtype=float)
        if float(np.linalg.norm(point - other)) < (min_inter_distance + recovery_margin * 0.5) - 1e-8:
            return False
        if downwash is not None:
            if is_in_downwash_zone(point, other, downwash) or is_in_downwash_zone(other, point, downwash):
                return False
    return True
