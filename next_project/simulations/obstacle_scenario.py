"""Obstacle scenario simulation orchestration."""

from __future__ import annotations

import time

import numpy as np

from .formation_simulation import FormationSimulation, SimulationConfig
from .runtime.collision_monitor import CollisionMonitor
from .runtime.fault_runtime import FaultRuntime
from .runtime.formation_adaptation_runtime import FormationAdaptationRuntime
from .runtime.planning_runtime import PlanningRuntime, make_planner
from .runtime.safety_metrics import (
    _collision_interval_count,
    _hard_collision_interval_count,
    _hard_collision_step_count,
    _min_airframe_signed_distance,
)
from .runtime.sensor_runtime import SensorRuntime
from core.formation_safety import (
    FormationSafetyConfig,
    downwash_zone,
    is_in_downwash_zone,
    min_inter_drone_distance,
)
from core.planning import DStarLite


class ObstacleScenarioSimulation(
    PlanningRuntime,
    SensorRuntime,
    FormationAdaptationRuntime,
    CollisionMonitor,
    FaultRuntime,
    FormationSimulation,
):
    def __init__(self, config: SimulationConfig):
        first_wp = config.waypoints[0] if config.waypoints else np.zeros(3)
        self.planned_trajectory = None
        self.planning_events: list[dict] = []
        self._sdf_query_count = 0
        self._clearance_check_count = 0
        super().__init__(config=config)
        arm_length = float(getattr(self.drone_params, "arm_length", 0.2))
        self._collision_margin = arm_length + config.safety_margin * config.detect_margin_scale
        self._setup_obstacles()
        self.formation_clearance_policy = self._make_formation_clearance_policy()
        self.formation_adaptation_policy = self._make_formation_adaptation_policy()
        self.formation_adaptation_events: list[dict] = []
        self._last_formation_adaptation_time: float | None = None
        self._last_lookahead_escape_time: float | None = None
        self._setup_planning()
        self._preflight_formation_adaptation_events = [dict(event) for event in self.formation_adaptation_events]
        self.formation_safety = FormationSafetyConfig(
            enabled=bool(getattr(config, "formation_safety_enabled", False)),
            min_inter_drone_distance=float(getattr(config, "formation_min_inter_drone_distance", 0.35)),
            downwash_radius=float(getattr(config, "formation_downwash_radius", 0.45)),
            downwash_height=float(getattr(config, "formation_downwash_height", 0.80)),
            conflict_vertical_step=max(
                float(getattr(config, "formation_downwash_height", 0.80)) * 0.5,
                0.25,
            ),
            conflict_lateral_step=max(
                float(getattr(config, "formation_min_inter_drone_distance", 0.35)) * 0.75,
                0.20,
            ),
        )
        self._formation_recovery_counts = [0] * int(getattr(config, "num_followers", 0))
        self._downwash_zone = downwash_zone(
            self.formation_safety.downwash_radius,
            self.formation_safety.downwash_height,
        )
        safe_first_wp = self._planning_waypoints[0] if getattr(self, "_planning_waypoints", None) else first_wp
        # 灏嗛鑸満涓庝粠鏈鸿捣濮嬩綅缃Щ鑷充慨姝ｅ悗鐨勭涓€涓埅鐐癸紝閬垮厤鍒濆鐬€佺┛瓒婇殰纰嶇墿
        self.leader.set_initial_state(safe_first_wp, [0.0, 0.0, 0.0])
        if hasattr(self, "followers"):
            initial_offsets = self.topology.get_offsets(self._active_formation_name())
            for follower, off in zip(self.followers, initial_offsets):
                safe_follower_start = self._safe_follower_target(
                    safe_first_wp,
                    safe_first_wp + off,
                    current_pos=None,
                )
                follower.set_initial_state(safe_follower_start, [0.0, 0.0, 0.0])
        # 鍒濆鍖栨敼杩涗汉宸ュ娍鍦烘硶閬跨锛堝弬鑰冭鏂?4.2.2 鑺傦級
        # 鑶ㄨ儉瑙勫垝 + 缂╁洖妫€娴嬶細瑙勫垝鑶ㄨ儉淇濇寔 1.5m锛屾娴嬮槇鍊煎彲鐙珛缂╂斁
        self.apf = self._build_apf()
        self.formation_apf = self._build_formation_apf()
        self.fault_detector = self._build_fault_detector()
        self.fault_log: list[dict] = []
        self._faulted_followers: set[int] = set()
        self._fault_injected = False
        # 浠庢満璧峰浣嶇疆涔熺浉搴旇皟鏁?        # 閬跨鐩稿叧鐘舵€?        self.collision_log: list[dict] = []
        self.replan_events: list[dict] = []
        self.sensor_logs: list[np.ndarray] = []
        self.executed_path: list[np.ndarray] = []


    def run(self) -> dict:
        """鎵ц鍚殰纰嶇墿鐨勭紪闃熶豢鐪熴€"""
        cfg = self.config
        steps = int(self.max_sim_time / self.dt) + 1
        follower_count = len(self.followers)

        history_time = np.zeros(steps, dtype=float)
        history_leader = np.zeros((steps, 3), dtype=float)
        history_followers = np.zeros((follower_count, steps, 3), dtype=float)
        target_positions = np.zeros((follower_count, steps, 3), dtype=float)
        error_vectors = np.zeros((follower_count, steps, 3), dtype=float)
        formation_errors = np.zeros((follower_count, steps), dtype=float)

        current_wp_idx = 0
        finished = False
        time_now = 0.0
        step_idx = 0
        leader_acc_filt = np.zeros(3, dtype=float)
        waypoint_events: list[dict] = []
        reached_waypoints: set[int] = set()
        dt = self.dt
        terminal_hold_pose: np.ndarray | None = None
        terminal_hold_steps = 0
        terminal_hold_required = max(8, int(round(0.15 / max(dt, 1e-6))))
        local_path: np.ndarray = np.array([self._planning_waypoints[0]], dtype=float)
        local_path_idx = 0
        local_path_task_idx = -1
        last_sensor_reading: np.ndarray | None = None
        last_sensor_time = -float("inf")
        sensor_period = max(float(getattr(cfg, "planner_replan_interval", dt)), dt)
        collision_check_steps = max(1, int(round(0.06 / max(dt, 1e-6))))

        wp_radius = cfg.wp_radius
        wp_radius_final = cfg.wp_radius_final
        waypoints_list = self.waypoints

        leader = self.leader
        leader_ctrl = self.leader_ctrl
        followers = self.followers
        follower_ctrls = self.follower_ctrls
        leader_wind = self.leader_wind
        winds = self.winds
        topology = self.topology
        alpha = cfg.leader_acc_alpha
        truth_obstacles = self.obstacles  # sensor + collision always use ground truth
        avoid_obstacles = self._discovered_obstacles if self._planner_initial_map_unknown else self.obstacles
        collision_margin = self._collision_margin
        leader_target_vel = np.zeros(3, dtype=float)

        self.collision_log = []
        self.replan_events = []
        self.sensor_logs = []
        self.executed_path = []
        self.fault_log = []
        self.formation_adaptation_events = [
            dict(event) for event in getattr(self, "_preflight_formation_adaptation_events", [])
        ]
        self._last_lookahead_escape_time = None
        self._faulted_followers = set()
        self._fault_injected = False
        self._formation_recovery_counts = [0] * follower_count

        while time_now < self.max_sim_time:
            self._maybe_switch_formation(time_now)
            if (
                cfg.fault_injection_enabled
                and not self._fault_injected
                and len(followers) > 0
                and time_now >= min(8.0, 0.25 * self.max_sim_time)
            ):
                followers[0].inject_fault(1, 0.3)
                self._fault_injected = True
                self.fault_log.append({
                    "t": float(time_now),
                    "type": "inject",
                    "drone": "follower_0",
                    "rotor": 1,
                    "severity": 0.3,
                })

            # 鍦ㄧ嚎閲嶈鍒掞紙浠?online 妯″紡锛?
            if cfg.planner_mode == "online" and hasattr(self, "replanner"):
                sensor_reading = None
                if self.sensor is not None:
                    leader_pos = leader.get_state()[0]
                    if time_now - last_sensor_time >= sensor_period:
                        last_sensor_reading = self.sensor.sense(leader_pos, truth_obstacles)
                        last_sensor_time = time_now
                        self.sensor_logs.append(last_sensor_reading.copy())
                    sensor_reading = last_sensor_reading

                    # 未知模式：每步持续将传感器读数注入栅格，保持动态障碍场最新
                    # BFS 3D 连通分量较重，限流 ≥0.25s 重建一次
                    if self._planner_initial_map_unknown and sensor_reading is not None:
                        changed = self.replanner._update_grid_from_sensor(leader_pos, sensor_reading)
                        if changed:
                            self.replanner.mark_sensor_grid_dirty()
                        if (
                            time_now - getattr(self, '_last_disc_rebuild', 0.0) >= 1.5
                            and self.replanner.consume_sensor_grid_dirty()
                        ):
                            self._update_discovered_obstacles()
                            self._last_disc_rebuild = time_now

                if getattr(cfg, "formation_adaptation_enabled", False):
                    channel_width = self._channel_width_from_sensor(sensor_reading)
                    event = self._apply_formation_adaptation(
                        time_now,
                        channel_width=channel_width,
                        clearance_margin=None,
                    )
                    if event is not None:
                        self._rebuild_planning_grid()
                elif getattr(cfg, "planner_use_formation_envelope", False):
                    channel_width = self._channel_width_from_sensor(sensor_reading)
                    if channel_width is not None:
                        previous = self.topology.current_formation
                        if self.topology.auto_shrink(channel_width):
                            if self.topology.current_formation != previous:
                                self._rebuild_planning_grid()

                leader_pos = leader.get_state()[0]
                task_goal = self._planning_waypoints[min(current_wp_idx, len(self._planning_waypoints) - 1)]
                # 在线重规划只服务于“到下一个任务航点”的局部执行，不再覆盖整个任务航点序列。
                target_goal = task_goal

                # D* Lite 鎯板垵濮嬪寲
                if self.replanner.incremental_planner is None and hasattr(self, "grid"):
                    try:
                        self.replanner.incremental_planner = DStarLite(
                            self.grid, leader_pos, target_goal
                        )
                    except Exception:
                        pass

                force_replan = local_path_task_idx != current_wp_idx
                path_remaining_dist = float("inf")
                if len(local_path) > 0:
                    path_remaining_dist = float(np.linalg.norm(local_path[-1] - leader_pos))
                local_path_exhausted = (
                    local_path_task_idx == current_wp_idx
                    and len(local_path) > 0
                    and np.linalg.norm(local_path[-1] - task_goal) > max(wp_radius, self.grid.resolution)
                    and path_remaining_dist < max(wp_radius * 1.5, cfg.leader_max_vel * 1.0)
                )
                planning_event_started = None
                planning_phase = None
                planning_planner = str(getattr(cfg, "planner_kind", ""))
                planning_fallback_reason = None
                new_path = self._maybe_rrt_lookahead_escape(
                    time_now,
                    leader_pos,
                    local_path,
                    target_goal,
                )
                if new_path is not None:
                    planning_phase = "online_lookahead_rrt_escape"
                    planning_planner = "rrt_star_escape"
                elif force_replan:
                    planning_event_started = time.perf_counter()
                    planning_phase = "online_force_replan"
                    planned_segment = self._planned_segment_for_task(leader_pos, target_goal)
                    if planned_segment is not None and self._segment_is_safe(planned_segment, collision_margin):
                        new_path = planned_segment
                        planning_planner = "planned_segment"
                    else:
                        planning_fallback_reason = "planned_segment_unavailable"
                        self.replanner._last_replan_time = -float("inf")
                        new_path = self.replanner.step(time_now, leader_pos, sensor_reading, target_goal)
                elif local_path_exhausted:
                    planning_event_started = time.perf_counter()
                    planning_phase = "online_exhausted_replan"
                    self.replanner._last_replan_time = -float("inf")
                    new_path = self.replanner.step(time_now, leader_pos, sensor_reading, target_goal)
                elif (
                    sensor_reading is not None
                    and len(sensor_reading) > 0
                    and float(np.min(sensor_reading)) < max(1.0, cfg.safety_margin * 3.0)
                ):
                    planning_event_started = time.perf_counter()
                    planning_phase = "online_sensor_replan"
                    new_path = self.replanner.step(time_now, leader_pos, sensor_reading, target_goal)
                else:
                    new_path = None
                # 未知模式：每次重规划后同步传感器发现的障碍物到动态障碍场
                if (
                    new_path is not None
                    and self._planner_initial_map_unknown
                    and self.replanner.consume_sensor_grid_dirty()
                ):
                    self._update_discovered_obstacles()
                if new_path is not None:
                    self.replan_events.extend(self.replanner.get_new_events())
                    accepted_path = self._accept_online_path(new_path, leader_pos, target_goal, time_now)
                    if planning_event_started is not None:
                        self.planning_events.append({
                            "t": float(time_now),
                            "phase": planning_phase,
                            "planner": planning_planner,
                            "wall_time_s": float(time.perf_counter() - planning_event_started),
                            "point_count": int(len(new_path)),
                            "accepted": accepted_path is not None,
                            "fallback_reason": planning_fallback_reason,
                        })
                    if accepted_path is not None:
                        local_path = np.asarray(accepted_path, dtype=float)
                        local_path_task_idx = current_wp_idx
                        # 在局部路径中找到当前位置最近点，从其后一点开始跟踪。
                        min_d = float("inf")
                        best_i = 0
                        for i, wp in enumerate(local_path):
                            d = float(np.linalg.norm(wp - leader_pos))
                            if d < min_d:
                                min_d = d
                                best_i = i
                        local_path_idx = min(best_i + 1, len(local_path) - 1)
                    elif force_replan:
                        local_path = np.array([leader_pos.copy()], dtype=float)
                        local_path_idx = 0
                        local_path_task_idx = current_wp_idx
                elif force_replan:
                    local_path = np.array([target_goal.copy()], dtype=float)
                    local_path_idx = 0
                    local_path_task_idx = current_wp_idx

            if terminal_hold_pose is not None:
                target_wp = terminal_hold_pose
            elif not finished:
                if cfg.planner_mode == "online" and len(local_path) > 0:
                    task_goal = self._planning_waypoints[min(current_wp_idx, len(self._planning_waypoints) - 1)]
                    target_wp, local_path_idx = self._select_online_target(
                        local_path,
                        leader.get_state()[0],
                        task_goal,
                        local_path_idx,
                    )
                else:
                    target_wp = waypoints_list[min(current_wp_idx, len(waypoints_list) - 1)]
            else:
                target_wp = self._planning_waypoints[-1] if self._planning_waypoints else waypoints_list[-1]

            wind_leader = leader_wind.sample(dt)
            leader_pos, leader_vel, _, _ = leader.get_state()
            if cfg.planner_mode == "online" and not finished and terminal_hold_pose is None:
                to_target = target_wp - leader_pos
                to_target_dist = float(np.linalg.norm(to_target))
                if to_target_dist > 1e-9:
                    speed_cmd = min(
                        float(cfg.leader_max_vel),
                        max(0.0, (to_target_dist - 0.25 * wp_radius) / max(dt * 8.0, 1e-6)),
                    )
                    leader_target_vel = (to_target / to_target_dist) * speed_cmd
                else:
                    leader_target_vel = np.zeros(3, dtype=float)
            else:
                leader_target_vel = np.zeros(3, dtype=float)
            follower_positions_now = [follower.get_state()[0] for follower in followers]
            offsets = topology.get_current_offsets(time_now)
            formation_leader_acc = np.zeros(3, dtype=float)
            formation_follower_accs = [np.zeros(3, dtype=float) for _ in followers]
            if self.formation_apf is not None and follower_positions_now:
                formation_leader_acc, formation_follower_accs = self.formation_apf.compute_formation_avoidance(
                    leader_pos=leader_pos,
                    follower_positions=follower_positions_now,
                    goal=target_wp,
                    obstacles=avoid_obstacles,
                    desired_offsets=offsets,
                )
            # 鏀硅繘 APF锛氶殰纰嶇墿鏂ュ姏锛堝惈鐩爣璺濈琛板噺 + 灞€閮ㄦ瀬灏忓€奸€冮€革級
            leader_sdf = float(avoid_obstacles.signed_distance(leader_pos))
            if leader_sdf < self.apf.r_rep:
                leader_repulsion_acc = self.apf.compute_avoidance_acceleration(
                    leader_pos, target_wp, avoid_obstacles)
            else:
                leader_repulsion_acc = np.zeros(3, dtype=float)
            leader_repulsion_acc = leader_repulsion_acc + formation_leader_acc
            leader_u = leader_ctrl.compute_control(
                leader.state,
                target_wp,
                target_vel=leader_target_vel,
                target_acc=leader_repulsion_acc,
            )
            leader.update_state(leader_u, wind=wind_leader)
            self._project_drone_state_to_safe(leader, collision_margin)
            leader_pos_new, leader_vel_new, _, _ = leader.get_state()
            self.executed_path.append(leader_pos_new.copy())

            # 领航机碰撞检测 (每步检查，与 C++ 对齐)
            if truth_obstacles.is_collision(leader_pos_new, inflate=collision_margin):
                self.collision_log.append({
                    "t": float(time_now),
                    "drone": "leader",
                    "pos": leader_pos_new.tolist(),
                })

            if not finished:
                dist_to_wp = np.linalg.norm(target_wp - leader_pos_new)
                if cfg.planner_mode == "online":
                    task_target = self._planning_waypoints[min(current_wp_idx, len(self._planning_waypoints) - 1)]
                    dist_to_task_wp = float(np.linalg.norm(task_target - leader_pos_new))
                    radius = wp_radius_final if current_wp_idx == len(self._planning_waypoints) - 1 else wp_radius
                    if current_wp_idx == len(self._planning_waypoints) - 1:
                        if dist_to_task_wp < radius:
                            terminal_hold_steps += 1
                            if terminal_hold_pose is None:
                                if current_wp_idx not in reached_waypoints:
                                    waypoint_events.append({
                                        "t": float(time_now),
                                        "type": "waypoint_reached",
                                        "index": int(current_wp_idx),
                                        "distance": float(dist_to_task_wp),
                                    })
                                    reached_waypoints.add(current_wp_idx)
                                terminal_hold_pose = leader_pos_new.copy()
                                leader_ctrl.reset()
                            if terminal_hold_steps >= terminal_hold_required:
                                finished = True
                        else:
                            terminal_hold_steps = 0
                    elif dist_to_task_wp < radius:
                        if current_wp_idx not in reached_waypoints:
                            waypoint_events.append({
                                "t": float(time_now),
                                "type": "waypoint_reached",
                                "index": int(current_wp_idx),
                                "distance": float(dist_to_task_wp),
                            })
                            reached_waypoints.add(current_wp_idx)
                        current_wp_idx += 1
                        local_path_idx = 0
                        local_path = np.array([leader_pos_new.copy()], dtype=float)
                        local_path_task_idx = -1
                else:
                    radius = wp_radius_final if current_wp_idx == len(waypoints_list) - 1 else wp_radius
                    if current_wp_idx == len(waypoints_list) - 1:
                        if dist_to_wp < radius:
                            terminal_hold_steps += 1
                            if terminal_hold_pose is None:
                                if current_wp_idx not in reached_waypoints:
                                    waypoint_events.append({
                                        "t": float(time_now),
                                        "type": "waypoint_reached",
                                        "index": int(current_wp_idx),
                                        "distance": float(dist_to_wp),
                                    })
                                    reached_waypoints.add(current_wp_idx)
                                terminal_hold_pose = leader_pos_new.copy()
                                leader_ctrl.reset()
                            if terminal_hold_steps >= terminal_hold_required:
                                finished = True
                        else:
                            terminal_hold_steps = 0
                    elif dist_to_wp < radius:
                        if current_wp_idx not in reached_waypoints:
                            waypoint_events.append({
                                "t": float(time_now),
                                "type": "waypoint_reached",
                                "index": int(current_wp_idx),
                                "distance": float(dist_to_wp),
                            })
                            reached_waypoints.add(current_wp_idx)
                        current_wp_idx += 1
                        if current_wp_idx >= len(waypoints_list):
                            finished = True

            leader_acc = (leader_vel_new - leader_vel) / dt
            leader_acc_filt = alpha * leader_acc + (1.0 - alpha) * leader_acc_filt

            reserved_targets: list[np.ndarray] = [leader_pos_new.copy()]
            reserved_actual_positions: list[np.ndarray] = [leader_pos_new.copy()]
            for i, follower in enumerate(followers):
                wind_follower = winds[i].sample(dt)
                follower_current_pos = follower.get_state()[0]
                nominal_target = leader_pos_new + offsets[i]
                target_pos = self._safe_follower_target(
                    leader_pos_new,
                    nominal_target,
                    current_pos=follower_current_pos,
                )
                target_pos = self._deconflict_follower_target(
                    leader_pos_new,
                    target_pos,
                    nominal_target,
                    reserved_targets,
                    i,
                    current_pos=follower_current_pos,
                )
                reserved_targets.append(np.asarray(target_pos, dtype=float))

                # 鏀硅繘 APF锛氶殰纰嶇墿鏂ュ姏 + 鏈洪棿鏂ュ姏
                # 鏀堕泦鍏朵粬浠庢満浣嶇疆鐢ㄤ簬鏈洪棿鏂ュ姏
                other_positions = []
                for j, other_follower in enumerate(followers):
                    if j != i:
                        other_positions.append(other_follower.get_state()[0])
                if float(avoid_obstacles.signed_distance(follower_current_pos)) < self.apf.r_rep or other_positions:
                    repulsion_acc = self.apf.compute_avoidance_acceleration(
                        follower_current_pos, target_pos, avoid_obstacles,
                        other_positions=other_positions)
                else:
                    repulsion_acc = np.zeros(3, dtype=float)
                if i < len(formation_follower_accs):
                    repulsion_acc = repulsion_acc + formation_follower_accs[i]

                follower_u = follower_ctrls[i].compute_control(
                    follower.state,
                    target_pos,
                    target_vel=leader_vel_new,
                    target_acc=leader_acc_filt + repulsion_acc,
                )
                if self.fault_detector is not None and i not in self._faulted_followers:
                    desired_state = np.zeros(6, dtype=float)
                    desired_state[0:3] = target_pos
                    desired_state[3:6] = leader_vel_new
                    if self.fault_detector.check(i, follower.state, desired_state, follower_u, control_max=20.0):
                        self._faulted_followers.add(i)
                        self.fault_log.append({
                            "t": float(time_now),
                            "type": "detect",
                            "drone": f"follower_{i}",
                        })
                        if getattr(cfg, "fault_reconfig_enabled", False):
                            topo = topology.fault_reconfigure(sorted(self._faulted_followers), transition_time=3.0)
                            self.fault_log.append({
                                "t": float(time_now),
                                "type": "reconfigure",
                                "topology": topo,
                                "failed": sorted(self._faulted_followers),
                            })
                follower.update_state(follower_u, wind=wind_follower)
                self._project_drone_state_to_safe(follower, collision_margin)
                if getattr(self, "formation_safety", None) is not None and self.formation_safety.enabled:
                    self._project_drone_state_from_neighbors(
                        follower,
                        reserved_actual_positions,
                        min_distance=self.formation_safety.min_inter_drone_distance,
                    )
                follower_pos = follower.get_state()[0]
                reserved_actual_positions.append(follower_pos.copy())

                # 从机碰撞检测 (每步检查，与 C++ 对齐)
                if truth_obstacles.is_collision(follower_pos, inflate=collision_margin):
                    self.collision_log.append({
                        "t": float(time_now),
                        "drone": f"follower_{i}",
                        "pos": follower_pos.tolist(),
                    })

                error_vec = follower_pos - target_pos
                target_positions[i, step_idx, :] = target_pos
                error_vectors[i, step_idx, :] = error_vec
                formation_errors[i, step_idx] = np.linalg.norm(error_vec)
                history_followers[i, step_idx, :] = follower_pos

            history_time[step_idx] = time_now
            history_leader[step_idx, :] = leader_pos_new

            step_idx += 1
            time_now += dt

        valid = slice(0, step_idx)
        errors = [formation_errors[i, valid] for i in range(follower_count)]
        metrics = {
            "mean": np.array([float(np.mean(err)) for err in errors], dtype=float),
            "max": np.array([float(np.max(err)) for err in errors], dtype=float),
            "final": np.array([float(err[-1]) for err in errors], dtype=float),
        }

        executed_arr = np.array(self.executed_path, dtype=float) if self.executed_path else np.zeros((0, 3), dtype=float)
        safety_metrics: dict[str, float | int] = {}
        if follower_count > 0:
            all_positions = [history_leader[valid, :]]
            all_positions.extend(history_followers[i, valid, :] for i in range(follower_count))
            min_pair_distance = float("inf")
            downwash_hits = 0
            for step in range(step_idx):
                poses = [history_leader[step, :]]
                poses.extend(history_followers[i, step, :] for i in range(follower_count))
                min_pair_distance = min(min_pair_distance, min_inter_drone_distance(poses))
                if getattr(self, "formation_safety", None) is not None and self.formation_safety.enabled:
                    for upper_i, upper in enumerate(poses):
                        for lower_i, lower in enumerate(poses):
                            if upper_i == lower_i:
                                continue
                            if is_in_downwash_zone(upper, lower, self._downwash_zone):
                                downwash_hits += 1
            safety_metrics = {
                "min_inter_drone_distance": float(min_pair_distance),
                "downwash_hits": int(downwash_hits),
            }
        if len(executed_arr) > 0:
            clearance_eval = self._make_formation_clearance_policy().evaluate_path(executed_arr)
            safety_metrics["formation_clearance"] = clearance_eval.to_dict()
            posthoc_eval = self._make_formation_clearance_policy(formation_aware=True).evaluate_path(executed_arr)
            safety_metrics["formation_clearance_posthoc"] = posthoc_eval.to_dict()

        map_knowledge = {
            "initial_map_unknown": bool(getattr(self.config, "planner_initial_map_unknown", False)),
            "truth_obstacle_count": int(len(list(self.obstacles))),
            "planner_static_occupied_count": None,
            "planner_sensor_occupied_count": None,
        }
        if getattr(self, "replanner", None) is not None:
            static_occupied = getattr(self.replanner, "_static_occupied", None)
            sensor_occupied = getattr(self.replanner, "_sensor_occupied", None)
            if static_occupied is not None:
                map_knowledge["planner_static_occupied_count"] = int(np.asarray(static_occupied, dtype=bool).sum())
            if sensor_occupied is not None:
                map_knowledge["planner_sensor_occupied_count"] = int(np.asarray(sensor_occupied, dtype=bool).sum())

        collision_interval_count = _collision_interval_count(self.collision_log, dt)
        hard_collision_steps = _hard_collision_step_count(self.collision_log, self.obstacles)
        hard_collision_intervals = _hard_collision_interval_count(self.collision_log, self.obstacles, dt)
        min_obs_sd = _min_airframe_signed_distance(
            executed_arr,
            [history_followers[i, valid, :] for i in range(follower_count)],
            self.obstacles,
        )
        collision_summary = {
            "collision_count": hard_collision_intervals,
            "collision_step_count": hard_collision_steps,
            "hard_collision_count": hard_collision_intervals,
            "hard_collision_step_count": hard_collision_steps,
            "clearance_warning_count": collision_interval_count,
            "clearance_warning_step_count": len(self.collision_log),
            "min_obstacle_signed_distance": float(min_obs_sd),
        }

        return {
            "time": history_time[valid],
            "leader": history_leader[valid, :],
            "followers": [history_followers[i, valid, :] for i in range(follower_count)],
            "targets": [target_positions[i, valid, :] for i in range(follower_count)],
            "error_vectors": [error_vectors[i, valid, :] for i in range(follower_count)],
            "errors": errors,
            "metrics": metrics,
            "completed_waypoint_count": len(self._planning_waypoints) if finished else current_wp_idx,
            "task_waypoints": np.array(self._task_waypoints, dtype=float),
            "waypoints": np.array(waypoints_list, dtype=float),
            "replanned_waypoints": np.array(local_path, dtype=float),
            "obstacles": self.obstacles,
            "planned_path": self.planned_path if self.planned_path is not None else executed_arr[:0],
            "planned_trajectory": None if self.planned_trajectory is None else self.planned_trajectory.to_dict(),
            "mpc_feasibility": None if self.mpc_feasibility is None else self.mpc_feasibility.to_dict(),
            "executed_path": executed_arr,
            "replan_events": self.replan_events,
            "planning_events": [self._planning_event_payload(event) for event in self.planning_events],
            "performance_counters": {
                "sdf_query_count": int(self._sdf_query_count),
                "clearance_check_count": int(self._clearance_check_count),
            },
            "waypoint_events": waypoint_events,
            "formation_adaptation_events": self.formation_adaptation_events,
            "sensor_logs": np.array(self.sensor_logs, dtype=float) if self.sensor_logs else None,
            "map_knowledge": map_knowledge,
            "collision_log": self.collision_log,
            "fault_log": self.fault_log,
            "safety_metrics": safety_metrics,
            "collision_summary": collision_summary,
        }

