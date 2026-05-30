from __future__ import annotations

import time

import numpy as np

from core.formation_adaptation import FormationAdaptationPolicy
from core.formation_safety import min_inter_drone_distance
from core.planning import RRTStar
from core.topology import FormationTopology


class FormationAdaptationRuntime:
    def _active_formation_name(self) -> str:
        if getattr(self.topology, "is_switching", False) and getattr(self.topology, "_target_formation", ""):
            return str(self.topology._target_formation)
        return str(getattr(self.topology, "current_formation", self.config.initial_formation))


    def _make_formation_adaptation_policy(self) -> FormationAdaptationPolicy:
        return FormationAdaptationPolicy(
            formations=tuple(getattr(self.config, "formation_adaptation_candidates", ("diamond", "v_shape", "triangle", "line"))),
            default_formation=str(self.config.initial_formation),
            min_hold_time_s=float(getattr(self.config, "formation_adaptation_min_hold_time", 1.0)),
            recovery_clearance_margin=float(getattr(self.config, "formation_adaptation_recovery_margin", 0.15)),
        )


    def _candidate_formation_envelopes(self) -> dict[str, tuple[float, float, float]]:
        envelopes: dict[str, tuple[float, float, float]] = {}
        spacing_by_formation = self._candidate_formation_spacings()
        for formation in getattr(self.config, "formation_adaptation_candidates", ("diamond", "v_shape", "triangle", "line")):
            try:
                topo = FormationTopology(
                    num_followers=self.config.num_followers,
                    spacing=spacing_by_formation.get(str(formation), self.topology.spacing),
                    arm_length=self.topology.arm_length,
                )
                envelopes[str(formation)] = topo.envelope_per_axis(str(formation))
            except Exception:
                continue
        return envelopes


    def _candidate_formation_spacings(self) -> dict[str, float]:
        guard = max(
            float(self.config.formation_spacing),
            float(getattr(self.config, "formation_min_inter_drone_distance", 0.35)) + 0.05,
            float(getattr(self.config, "formation_downwash_radius", 0.45)),
        )
        return {
            str(formation): guard
            for formation in getattr(self.config, "formation_adaptation_candidates", ("diamond", "v_shape", "triangle", "line"))
        }


    def _candidate_formation_min_pair_distances(self) -> dict[str, float]:
        distances: dict[str, float] = {}
        spacing_by_formation = self._candidate_formation_spacings()
        for formation, spacing in spacing_by_formation.items():
            try:
                topo = FormationTopology(
                    num_followers=self.config.num_followers,
                    spacing=float(spacing),
                    arm_length=self.topology.arm_length,
                )
                offsets = topo.get_offsets(formation)
                distances[formation] = min_inter_drone_distance([np.zeros(3, dtype=float), *offsets])
            except Exception:
                continue
        return distances


    def _apply_formation_adaptation(
        self,
        time_now: float,
        channel_width: tuple[float, float, float] | None,
        clearance_margin: float | None,
    ) -> dict | None:
        if not getattr(self.config, "formation_adaptation_enabled", False):
            return None
        decision = self.formation_adaptation_policy.decide(
            time_now=float(time_now),
            current_formation=self._active_formation_name(),
            channel_width=channel_width,
            envelope_by_formation=self._candidate_formation_envelopes(),
            spacing_by_formation=self._candidate_formation_spacings(),
            min_pair_distance_by_formation=self._candidate_formation_min_pair_distances(),
            min_inter_drone_distance=float(getattr(self.config, "formation_min_inter_drone_distance", 0.35)),
            clearance_margin=clearance_margin,
            last_switch_time_s=self._last_formation_adaptation_time,
        )
        if not decision.should_switch:
            return None
        if decision.target_spacing is not None:
            self.topology.spacing = float(decision.target_spacing)
            self.config.formation_spacing = float(decision.target_spacing)
        if float(time_now) <= 0.0:
            self._set_topology_formation_immediate(decision.target_formation)
        else:
            self.topology.switch_formation(
                decision.target_formation,
                transition_time=float(getattr(self.config, "formation_adaptation_transition_time", 1.5)),
            )
        self._last_formation_adaptation_time = float(time_now)
        event = decision.to_event(float(time_now))
        self.formation_adaptation_events.append(event)
        return event


    def _set_topology_formation_immediate(self, formation: str) -> None:
        offsets = self.topology.get_offsets(formation)
        self.topology._current_offsets = [offset.copy() for offset in offsets]
        self.topology.current_formation = str(formation)
        self.topology._target_formation = ""
        self.topology._source_offsets = []
        self.topology._target_offsets = []
        self.topology._switching = False
        self.topology._switch_start_time = 0.0
        self.topology._transition_time = 0.0


    def _maybe_preplan_formation_adaptation(self) -> None:
        if not getattr(self.config, "formation_adaptation_enabled", False):
            return
        if not self._formation_clearance_enabled() or len(self._planning_waypoints) < 2:
            return
        reference = np.asarray(self._planning_waypoints, dtype=float)
        evaluation = self._make_formation_clearance_policy(formation_aware=True).evaluate_path(reference)
        if evaluation.min_formation_margin >= 0.0:
            return
        event = self._apply_formation_adaptation(
            0.0,
            channel_width=None,
            clearance_margin=float(evaluation.min_formation_margin),
        )
        if event is not None:
            self._rebuild_planning_grid()


    def _path_window_from_position(
        self,
        path: np.ndarray,
        position: np.ndarray,
        lookahead_distance: float | None = None,
    ) -> np.ndarray:
        """Return the local path slice inside the formation lookahead horizon."""
        path = np.asarray(path, dtype=float)
        position = np.asarray(position, dtype=float)
        if len(path) == 0:
            return np.zeros((0, 3), dtype=float)
        if len(path) == 1:
            return path.copy()

        horizon = float(
            getattr(self.config, "formation_lookahead_distance", 4.0)
            if lookahead_distance is None else lookahead_distance
        )
        closest_i = int(np.argmin(np.linalg.norm(path - position, axis=1)))
        window: list[np.ndarray] = [position.copy()]
        distance = 0.0
        prev = position.copy()
        for point in path[closest_i + 1:]:
            point = np.asarray(point, dtype=float)
            seg_len = float(np.linalg.norm(point - prev))
            if seg_len < 1e-9:
                prev = point
                continue
            if distance + seg_len >= horizon:
                ratio = max(0.0, min(1.0, (horizon - distance) / seg_len))
                window.append(prev + (point - prev) * ratio)
                break
            window.append(point.copy())
            distance += seg_len
            prev = point
        if len(window) == 1:
            window.append(path[min(closest_i + 1, len(path) - 1)].copy())
        return np.asarray(window, dtype=float)


    @staticmethod
    def _path_max_turn_angle(path: np.ndarray) -> float:
        """Return the largest turn angle in radians for adjacent path segments."""
        path = np.asarray(path, dtype=float)
        if len(path) < 3:
            return 0.0
        max_angle = 0.0
        for idx in range(1, len(path) - 1):
            left = path[idx] - path[idx - 1]
            right = path[idx + 1] - path[idx]
            left_norm = float(np.linalg.norm(left))
            right_norm = float(np.linalg.norm(right))
            if left_norm < 1e-9 or right_norm < 1e-9:
                continue
            cos_angle = float(np.dot(left, right) / (left_norm * right_norm))
            angle = float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
            max_angle = max(max_angle, angle)
        return max_angle


    def _lookahead_path_diagnosis(self, path: np.ndarray, position: np.ndarray) -> dict:
        """Diagnose whether the forward path window needs early formation handling."""
        path = np.asarray(path, dtype=float)
        window = self._path_window_from_position(path, position)
        max_turn = self._path_max_turn_angle(window)
        turn_threshold = float(getattr(self.config, "formation_lookahead_turn_threshold_rad", 1.05))
        min_clearance = self._formation_clearance_base()
        clearance = self._path_segment_clearance(window, min_clearance) if len(window) >= 2 else float("inf")
        clearance_margin = clearance - min_clearance

        blocked = False
        reason = "lookahead_clear"
        if clearance_margin < 0.0:
            blocked = True
            reason = "lookahead_reference_blocked"
        if max_turn >= turn_threshold:
            blocked = True
            reason = "lookahead_sharp_turn"

        return {
            "blocked": bool(blocked),
            "reason": reason,
            "window_point_count": int(len(window)),
            "max_turn_angle_rad": float(max_turn),
            "clearance": float(clearance),
            "clearance_margin": float(clearance_margin),
        }


    def _plan_rrt_escape_path(
        self,
        leader_pos: np.ndarray,
        task_goal: np.ndarray,
        min_clearance: float,
        time_now: float,
        *,
        reason: str = "lookahead_reference_blocked",
        candidate_goals: list[tuple[str, np.ndarray]] | None = None,
    ) -> np.ndarray | None:
        """Use a short RRT* attempt to escape a blocked or misleading local window."""
        leader_pos = np.asarray(leader_pos, dtype=float)
        task_goal = np.asarray(task_goal, dtype=float)
        min_clearance = float(min_clearance)
        goal_options: list[tuple[str, np.ndarray]] = [("task_goal", task_goal)]
        if candidate_goals:
            goal_options.extend((str(kind), np.asarray(goal, dtype=float)) for kind, goal in candidate_goals)
        unique_goals: list[tuple[str, np.ndarray]] = []
        seen_goals: set[tuple[float, float, float]] = set()
        for kind, goal in goal_options:
            if float(np.linalg.norm(goal - leader_pos)) < max(self.grid.resolution, 1e-6):
                continue
            key = tuple(np.round(goal, 3))
            if key in seen_goals:
                continue
            seen_goals.add(key)
            unique_goals.append((kind, goal))

        attempt_event = {
            "t": float(time_now),
            "kind": "rrt_escape_attempt",
            "from": self._active_formation_name(),
            "to": self._active_formation_name(),
            "reason": reason,
            "planner": "rrt_star_escape",
            "goal_count": int(len(unique_goals)),
        }
        self.formation_adaptation_events.append(attempt_event)

        planner_name = "rrt_star_escape"
        for goal_kind, goal in unique_goals:
            candidate: np.ndarray | None = None
            planner_name = "rrt_star_escape"
            try:
                planner = RRTStar(
                    max_iter=int(getattr(self.config, "formation_lookahead_rrt_max_iter", 800)),
                    rewire_radius=float(getattr(self.config, "formation_lookahead_rrt_rewire_radius", 1.2)),
                    smooth_method="shortcut",
                )
                candidate = np.asarray(planner.plan(leader_pos, goal, self.grid, seed=42), dtype=float)
            except Exception:
                direct = np.vstack([leader_pos, goal])
                if self._segment_is_safe(direct, min_clearance):
                    candidate = direct
                    planner_name = "direct_clear_escape"

            if candidate is not None and len(candidate) >= 2 and self._segment_is_safe(candidate, min_clearance):
                self.formation_adaptation_events.append({
                    "t": float(time_now),
                    "kind": "rrt_escape_accepted",
                    "from": self._active_formation_name(),
                    "to": self._active_formation_name(),
                    "reason": reason,
                    "planner": planner_name,
                    "goal_kind": goal_kind,
                    "goal": np.asarray(goal, dtype=float).tolist(),
                    "point_count": int(len(candidate)),
                })
                return np.asarray(candidate, dtype=float)

        self.formation_adaptation_events.append({
            "t": float(time_now),
            "kind": "rrt_escape_failed",
            "from": self._active_formation_name(),
            "to": self._active_formation_name(),
            "reason": reason,
            "planner": planner_name,
            "goal_count": int(len(unique_goals)),
            "point_count": 0,
        })
        return None


    def _lookahead_escape_goal_candidates(
        self,
        local_path: np.ndarray,
        leader_pos: np.ndarray,
        task_goal: np.ndarray,
        min_clearance: float,
    ) -> list[tuple[str, np.ndarray]]:
        """Build local RRT escape subgoals when the final task goal is temporarily unreachable."""
        local_path = np.asarray(local_path, dtype=float)
        leader_pos = np.asarray(leader_pos, dtype=float)
        task_goal = np.asarray(task_goal, dtype=float)
        candidates: list[tuple[str, np.ndarray]] = []
        window = self._path_window_from_position(local_path, leader_pos)
        if len(window) >= 2:
            candidates.append(("lookahead_window_end", window[-1].copy()))

        if len(local_path) >= 2:
            closest_i = int(np.argmin(np.linalg.norm(local_path - leader_pos, axis=1)))
            for idx in np.linspace(closest_i + 1, len(local_path) - 1, num=min(3, max(1, len(local_path) - closest_i - 1))):
                point = local_path[int(round(idx))]
                candidates.append(("reference_future_point", np.asarray(point, dtype=float)))

        enriched: list[tuple[str, np.ndarray]] = []
        seen: set[tuple[float, float, float]] = set()
        for kind, goal in candidates:
            goal = np.asarray(goal, dtype=float)
            options = [(kind, goal)]
            try:
                projected = self._project_to_planning_free(
                    goal,
                    prefer=goal - leader_pos,
                    min_clearance=min_clearance,
                    max_radius_m=max(2.0, float(getattr(self.config, "formation_lookahead_distance", 4.0))),
                )
                if float(np.linalg.norm(projected - goal)) > 1e-6:
                    options.append((f"{kind}_projected", projected))
            except Exception:
                pass
            for option_kind, option_goal in options:
                if float(np.linalg.norm(option_goal - leader_pos)) < max(self.grid.resolution, 1e-6):
                    continue
                key = tuple(np.round(option_goal, 3))
                if key in seen:
                    continue
                seen.add(key)
                enriched.append((option_kind, option_goal.copy()))
        return enriched


    def _maybe_rrt_lookahead_escape(
        self,
        time_now: float,
        leader_pos: np.ndarray,
        local_path: np.ndarray,
        task_goal: np.ndarray,
    ) -> np.ndarray | None:
        """Run early lookahead diagnosis and RRT escape before regular online replanning."""
        if not getattr(self.config, "formation_lookahead_enabled", False):
            return None
        if not getattr(self.config, "formation_lookahead_rrt_enabled", False):
            return None
        local_path = np.asarray(local_path, dtype=float)
        if len(local_path) < 2:
            return None
        min_interval = float(getattr(self.config, "formation_lookahead_min_interval", 0.8))
        if (
            self._last_lookahead_escape_time is not None
            and float(time_now) - float(self._last_lookahead_escape_time) < min_interval
        ):
            return None

        diagnosis = self._lookahead_path_diagnosis(local_path, leader_pos)
        if not bool(diagnosis.get("blocked", False)):
            return None

        min_clearance = self._formation_clearance_base()
        self.formation_adaptation_events.append({
            "t": float(time_now),
            "kind": "lookahead_reference_blocked",
            "from": self._active_formation_name(),
            "to": self._active_formation_name(),
            "reason": str(diagnosis.get("reason") or "lookahead_reference_blocked"),
            "clearance_margin": float(diagnosis.get("clearance_margin", 0.0)),
            "max_turn_angle_rad": float(diagnosis.get("max_turn_angle_rad", 0.0)),
        })

        if getattr(self.config, "formation_adaptation_enabled", False):
            event = self._apply_formation_adaptation(
                time_now,
                channel_width=None,
                clearance_margin=float(diagnosis.get("clearance_margin", 0.0)),
            )
            if event is not None:
                self._rebuild_planning_grid()

        started = time.perf_counter()
        candidate = self._plan_rrt_escape_path(
            leader_pos,
            task_goal,
            min_clearance,
            time_now,
            reason=str(diagnosis.get("reason") or "lookahead_reference_blocked"),
            candidate_goals=self._lookahead_escape_goal_candidates(
                local_path,
                leader_pos,
                task_goal,
                min_clearance,
            ),
        )
        self._last_lookahead_escape_time = float(time_now)
        self.planning_events.append({
            "t": float(time_now),
            "phase": "online_lookahead_rrt_escape",
            "planner": "rrt_star_escape",
            "wall_time_s": float(time.perf_counter() - started),
            "point_count": int(len(candidate)) if candidate is not None else 0,
            "accepted": candidate is not None,
            "fallback_reason": None if candidate is not None else "rrt_escape_failed",
        })
        return None if candidate is None else np.asarray(candidate, dtype=float)
