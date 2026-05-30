from __future__ import annotations

import numpy as np

from core.formation_safety import (
    deconflict_follower_target,
    follower_safety_correction,
    nominal_target_ready_for_recovery,
)


class CollisionMonitor:
    def _safe_follower_target(
        self,
        leader_pos: np.ndarray,
        raw_target: np.ndarray,
        current_pos: np.ndarray | None = None,
    ) -> np.ndarray:
        """将从机编队目标约束到安全且尽量保持队形的位置。"""
        leader_pos = np.asarray(leader_pos, dtype=float)
        raw_target = np.asarray(raw_target, dtype=float)
        current = None if current_pos is None else np.asarray(current_pos, dtype=float)
        min_clearance = max(float(self._collision_margin) + 0.05, float(self.config.safety_margin))

        if getattr(self, "formation_safety", None) is not None and self.formation_safety.enabled:
            return follower_safety_correction(
                leader_pos,
                raw_target,
                current_pos=current,
                signed_distance=lambda p: float(self.obstacles.signed_distance(np.asarray(p, dtype=float))),
                segment_is_safe=lambda path, clearance: bool(self._segment_is_safe(np.asarray(path, dtype=float), float(clearance))),
                project_to_free=self._project_to_planning_free,
                min_clearance=min_clearance,
                shrink_steps=self.formation_safety.follower_shrink_steps,
            )

        candidates: list[np.ndarray] = []
        # 优先尝试完整队形，再逐步向 leader 收缩，保证窄通道/近障碍时不把从机目标放进障碍物。
        for scale in np.linspace(1.0, 0.0, 11):
            candidates.append(leader_pos + (raw_target - leader_pos) * float(scale))

        try:
            projected = self._project_to_planning_free(
                raw_target,
                prefer=leader_pos - raw_target,
                min_clearance=min_clearance,
                max_radius_m=max(2.0, np.linalg.norm(raw_target - leader_pos) + 1.0),
            )
            candidates.insert(1, projected)
        except Exception:
            pass

        seen: set[tuple[float, float, float]] = set()
        for candidate in candidates:
            candidate = np.asarray(candidate, dtype=float)
            key = tuple(np.round(candidate, 4))
            if key in seen:
                continue
            seen.add(key)
            if float(self.obstacles.signed_distance(candidate)) < min_clearance - 1e-8:
                continue
            if current is not None:
                segment = np.vstack([current, candidate])
                if not self._segment_is_safe(segment, min_clearance):
                    continue
            return candidate.copy()

        if current is not None and float(self.obstacles.signed_distance(current)) >= 0.0:
            return current.copy()
        return leader_pos.copy()


    def _deconflict_follower_target(
        self,
        leader_pos: np.ndarray,
        candidate_target: np.ndarray,
        nominal_target: np.ndarray,
        reserved_positions: list[np.ndarray],
        follower_idx: int,
        current_pos: np.ndarray | None = None,
    ) -> np.ndarray:
        """对已安全化的 follower 目标再做机间去冲突修正。"""
        if not (getattr(self, "formation_safety", None) is not None and self.formation_safety.enabled):
            return np.asarray(candidate_target, dtype=float)
        min_clearance = max(float(self._collision_margin) + 0.05, float(self.config.safety_margin))
        nominal_target = np.asarray(nominal_target, dtype=float)
        reserved = [np.asarray(p, dtype=float) for p in reserved_positions]
        current = None if current_pos is None else np.asarray(current_pos, dtype=float)
        if nominal_target_ready_for_recovery(
            nominal_target,
            reserved_positions=reserved,
            current_pos=current,
            signed_distance=lambda p: float(self.obstacles.signed_distance(np.asarray(p, dtype=float))),
            segment_is_safe=lambda path, clearance: bool(self._segment_is_safe(np.asarray(path, dtype=float), float(clearance))),
            min_clearance=min_clearance,
            min_inter_distance=self.formation_safety.min_inter_drone_distance,
            downwash=self._downwash_zone,
            recovery_margin=self.formation_safety.recovery_clearance_margin,
        ):
            self._formation_recovery_counts[follower_idx] += 1
            if self._formation_recovery_counts[follower_idx] >= self.formation_safety.recovery_hold_steps:
                return nominal_target.copy()
        else:
            self._formation_recovery_counts[follower_idx] = 0

        preferred_target = (
            nominal_target
            if self._formation_recovery_counts[follower_idx] >= self.formation_safety.recovery_hold_steps
            else None
        )
        return deconflict_follower_target(
            candidate_target,
            leader_pos=np.asarray(leader_pos, dtype=float),
            reserved_positions=reserved,
            current_pos=current,
            signed_distance=lambda p: float(self.obstacles.signed_distance(np.asarray(p, dtype=float))),
            segment_is_safe=lambda path, clearance: bool(self._segment_is_safe(np.asarray(path, dtype=float), float(clearance))),
            min_clearance=min_clearance,
            min_inter_distance=self.formation_safety.min_inter_drone_distance,
            downwash=self._downwash_zone,
            vertical_step=self.formation_safety.conflict_vertical_step,
            lateral_step=self.formation_safety.conflict_lateral_step,
            preferred_target=preferred_target,
        )


    def _project_drone_state_to_safe(self, drone, min_clearance: float) -> bool:
        """执行层安全屏障：若动力学积分后进入安全边界，则投影回自由侧。"""
        pos = drone.state[0:3].copy()
        sd = float(self.obstacles.signed_distance(pos))
        if sd >= min_clearance:
            return False

        eps = 0.03
        grad = np.array([
            self.obstacles.signed_distance(pos + [eps, 0.0, 0.0])
            - self.obstacles.signed_distance(pos - [eps, 0.0, 0.0]),
            self.obstacles.signed_distance(pos + [0.0, eps, 0.0])
            - self.obstacles.signed_distance(pos - [0.0, eps, 0.0]),
            self.obstacles.signed_distance(pos + [0.0, 0.0, eps])
            - self.obstacles.signed_distance(pos - [0.0, 0.0, eps]),
        ], dtype=float) / (2.0 * eps)
        grad_norm = float(np.linalg.norm(grad))
        if grad_norm < 1e-9:
            corrected = self._project_to_planning_free(
                pos,
                min_clearance=min_clearance,
                max_radius_m=max(1.0, min_clearance + 0.8),
            )
            normal = corrected - pos
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm < 1e-9:
                return False
            normal = normal / normal_norm
        else:
            normal = grad / grad_norm
            corrected = pos + normal * (min_clearance - sd + 1e-3)

        drone.state[0:3] = corrected
        normal_vel = float(np.dot(drone.state[3:6], normal))
        if normal_vel < 0.0:
            drone.state[3:6] = drone.state[3:6] - normal_vel * normal
        return True


    def _project_drone_state_from_neighbors(
        self,
        drone,
        reserved_positions: list[np.ndarray],
        *,
        min_distance: float,
    ) -> bool:
        """执行层机间距屏障：若状态积分后过近，则沿相对方向投影到安全距离。"""
        pos = drone.state[0:3].copy()
        min_distance = float(min_distance)
        projected = False
        for other in reserved_positions:
            other = np.asarray(other, dtype=float)
            delta = pos - other
            dist = float(np.linalg.norm(delta))
            if dist >= min_distance - 1e-9:
                continue
            if dist < 1e-9:
                normal = np.array([1.0, 0.0, 0.0], dtype=float)
            else:
                normal = delta / dist
            pos = other + normal * (min_distance + 1e-6)
            normal_vel = float(np.dot(drone.state[3:6], normal))
            if normal_vel < 0.0:
                drone.state[3:6] = drone.state[3:6] - normal_vel * normal
            projected = True
        if projected:
            drone.state[0:3] = pos
        return projected


    def _compute_obstacle_repulsion(self, position: np.ndarray, influence_distance: float = 2.0,
                                    max_repulsion: float = 1.5) -> np.ndarray:
        """璁＄畻闅滅鐗╂帓鏂ュ姞閫熷害锛岀敤浜庡疄鏃堕伩闅溿€?
        鍘熺悊
        ----
        浣跨敤绗﹀彿璺濈鍦烘暟鍊兼搴︿及璁℃帓鏂ユ柟鍚戯紝鎺掓枼鍔涘ぇ灏忛殢璺濈鍑忓皬鑰?        浜屾澧為暱锛屼粎鍦?influence_distance 鑼冨洿鍐呯敓鏁堛€?        杩斿洖鐨勫悜閲忛噺绾蹭负 m/s虏锛堝墠棣堝姞閫熷害锛夛紝鍙犲姞鍒版帶鍒跺櫒鐨?`target_acc`
        鑰岄潪鐩爣浣嶇疆涓婏紝閬垮厤浣嶇疆鐜ぇ璺冲彉瀵艰嚧 PID 璺熻釜宕╂簝銆?        """
        sd = self.obstacles.signed_distance(position)
        if sd >= influence_distance:
            return np.zeros(3, dtype=float)

        eps = 0.05
        grad = np.array([
            self.obstacles.signed_distance(position + [eps, 0, 0])
            - self.obstacles.signed_distance(position - [eps, 0, 0]),
            self.obstacles.signed_distance(position + [0, eps, 0])
            - self.obstacles.signed_distance(position - [0, eps, 0]),
            self.obstacles.signed_distance(position + [0, 0, eps])
            - self.obstacles.signed_distance(position - [0, 0, eps]),
        ], dtype=float) / (2.0 * eps)

        grad_norm = float(np.linalg.norm(grad))
        if grad_norm < 1e-10:
            return np.zeros(3, dtype=float)
        grad /= grad_norm

        # 浜屾琛板噺鍔犻€熷害锛岄噺绾?m/s虏
        force_mag = max_repulsion * (1.0 - sd / influence_distance) ** 2
        return grad * force_mag
