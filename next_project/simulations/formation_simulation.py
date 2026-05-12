"""编队仿真示例。

用途
----
把动力学、控制器、风场和编队拓扑组合成一个完整的领航-跟随仿真流程，
并输出可用于评测与可视化的数据。

原理
----
1) 领航机按航点执行位置跟踪控制。
2) 从机目标位置由下式给出：

    p_i^*(t) = p_L(t) + Delta_i(t)

    其中 p_L(t) 是领航机位置，Delta_i(t) 是第 i 架从机编队偏移。

3) 从机控制采用速度和加速度前馈：

    u_i = C(x_i, p_i^*, v_L, a_L)

    其中 C(·) 为串级控制器，v_L 和 a_L 分别为领航机速度和加速度。

4) 为减小前馈噪声对从机的抖动，使用一阶低通滤波：

    a_f(k) = alpha * a_L(k) + (1 - alpha) * a_f(k-1)


"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from core.drone import Drone
from core.drone_params import get_drone_params
from core.controller import Controller, HybridAttitudeController, BacksteppingController, GeometricSE3Controller
from core.wind_field import WindField
from core.topology import FormationTopology


@dataclass
class SimulationConfig:
    """仿真配置。

    用途
    ----
    集中管理步长、控制器参数、航点判据和拓扑切换事件，避免硬编码。

    原理
    ----
    通过配置驱动仿真，保证不同实验仅改变配置项即可复现实验结果。
    """

    dt: float = 0.012
    max_sim_time: float = 30.0
    use_smc: bool = True

    # 编队规模与初始队形配置。
    num_followers: int = 3
    formation_spacing: float = 2.0
    initial_formation: str = "v_shape"

    # 通过限制领航机速度和加速度，降低急转和瞬态冲击。
    leader_gain_scale: float = 0.9
    leader_max_vel: float = 4.0
    leader_max_acc: float = 5.0

    use_backstepping: bool = False  # 是否使用反步法位置环 + SMC 姿态环
    follower_gain_scale: float = 1.0
    follower_max_vel: float = 10.0
    follower_max_acc: float = 10.0

    leader_wind_seed: int = 42
    follower_wind_seed_start: int = 100
    wind_tau: float = 0.25

    # 放宽普通航点切换半径，减少尖锐转折导致的编队瞬态误差。
    wp_radius: float = 1.5
    wp_radius_final: float = 0.3

    # 低通滤波系数，平滑主机加速度前馈。
    leader_acc_alpha: float = 0.2

    # (触发时间, 目标队形, 过渡时长)
    formation_schedule: list[tuple[float, str, float]] = field(default_factory=list)

    # 当 initial_formation="custom" 时生效，长度需与 num_followers 一致。
    custom_initial_offsets: list[np.ndarray] | None = None

    # 默认航点
    waypoints: list[np.ndarray] = field(default_factory=lambda: [
        np.array([0.0, 0.0, 0.0], dtype=float),
        np.array([0.0, 0.0, 15.0], dtype=float),
        np.array([20.0, 0.0, 15.0], dtype=float),
        np.array([20.0, 20.0, 15.0], dtype=float),
        np.array([0.0, 20.0, 15.0], dtype=float),
        np.array([0.0, 0.0, 0.0], dtype=float),
    ])

    # ---- 路径规划与避障 ----
    enable_obstacles: bool = False
    obstacle_field: object | None = None  # ObstacleField 实例
    map_file: str | None = None           # 优先级高于 obstacle_field

    planner_kind: str = "astar"           # "astar" | "turn_constrained_astar" | "hybrid_astar" | "dijkstra" | "rrt_star"
    planner_mode: str = "offline"         # "offline" | "online"
    planner_resolution: float = 0.5       # 栅格分辨率 m
    planner_replan_interval: float = 0.4  # 在线重规划周期 s
    planner_horizon: float = 6.0          # 在线规划前瞻距离 m
    planner_max_iter: int = 4000          # RRT*/InformedRRT* 最大迭代次数
    planner_rewire_radius: float = 1.5    # RRT* 重连半径
    safety_margin: float = 0.3            # 安全裕度 m
    planner_sdf_aware: bool = True        # 是否启用 SDF 感知栅格（捕获薄障碍物）
    planner_esdf_aware: bool = True       # 是否启用 ESDF 软代价（路径自动远离障碍物）
    planner_use_formation_envelope: bool = False  # 是否用完整编队包络膨胀规划栅格
    planner_z_bounds: tuple[float, float] | None = None  # 可选规划高度范围 [z_min, z_max]
    plan_clearance_extra: float = 0.0     # 规划时额外推远距离 m
    detect_margin_scale: float = 0.5      # 检测阈值缩放: arm_length + safety_margin*scale
                                           #   1.0=严格(0.5m), 0.5=宽松(0.35m), 0.0=仅臂长(0.2m)

    sensor_enabled: bool = False          # 是否启用机载测距
    sensor_max_range: float = 8.0
    sensor_noise_std: float = 0.02
    sensor_directions: int = 6            # ±x, ±y, ±z

    # ---- 论文2: 双模式 GNN ----
    danger_mode_enabled: bool = False
    gnn_V: float = 100.0
    gnn_E: float = 50.0
    gnn_alpha: float = 2.0
    gnn_beta: float = 2.0
    gnn_gamma: float = 2.0
    gnn_angular_res: int = 8
    gnn_buffer_zone: float = 0.3

    # ---- 论文1: 改进 APF（profile 化，C5 修补） ----
    apf_paper1_profile: str = "off"  # "off" | "conservative" | "aggressive"
    apf_comm_range: float = 10.0
    apf_centroid_alpha: float = 0.4
    apf_centroid_beta: float = 0.6
    # 高级开发开关（profile 之外的细粒度覆盖）
    apf_dev_override: bool = False
    apf_adaptive_n_decay: bool = False
    apf_formation_centroid: bool = False
    apf_comm_constraint: bool = False
    apf_rotational_escape: bool = False

    # ---- 论文3: 风险驱动重规划间隔 ----
    replan_adaptive_interval: bool = False
    replan_interval_min: float = 0.1
    replan_interval_max: float = 1.0

    # ---- 论文4: 容错 ----
    fault_injection_enabled: bool = False
    fault_detection_enabled: bool = False
    fault_reconfig_enabled: bool = False
    # FaultDetector 阈值（C3 修补：配置化，禁止硬编码）
    fault_detector_max_acc: float = 10.0  # m/s²
    fault_detector_pos_dev: float = 5.0
    fault_detector_saturate_steps: int = 50

    # ---- 双模式阈值 ----
    sensor_danger_threshold: float = 2.0
    sensor_safe_threshold: float = 4.0
    sdf_danger_threshold: float = 0.5

    # ---- S5: Safety Profile / Risk Report ----
    safety_profile: str = "indoor_demo"

    # ---- S4: Formation safety ----
    formation_safety_enabled: bool = False
    formation_min_inter_drone_distance: float = 0.35
    formation_downwash_radius: float = 0.45
    formation_downwash_height: float = 0.80
    formation_adaptation_enabled: bool = False
    formation_adaptation_candidates: tuple[str, ...] = ("diamond", "v_shape", "triangle", "line")
    formation_adaptation_min_hold_time: float = 1.0
    formation_adaptation_recovery_margin: float = 0.15
    formation_adaptation_transition_time: float = 1.5

    # ---- S2: DroneParams profile ----
    drone_profile: str = "default_1kg"

    # ---- S1: Trajectory optimizer ----
    trajectory_optimizer_enabled: bool = False
    trajectory_optimizer_method: str = "moving_average"
    trajectory_optimizer_nominal_speed: float = 1.0
    trajectory_optimizer_sample_dt: float = 0.2

    # ---- S3: Controller profile ----
    controller_kind: str = "pid_smc"

class FormationSimulation:
    """领航-跟随编队仿真器。

    用途
    ----
    执行单次编队仿真并输出时间序列、误差序列与统计指标。

    原理
    ----
    每个离散时刻执行“领航机更新 -> 从机更新 -> 误差记录”的同步闭环。
    """

    def __init__(self, config: SimulationConfig | None = None,
                 dt: float | None = None, max_sim_time: float | None = None,
                 use_smc: bool | None = None):
        self.config = config if config is not None else SimulationConfig()

        # 兼容旧接口参数。
        if dt is not None:
            self.config.dt = float(dt)
        if max_sim_time is not None:
            self.config.max_sim_time = float(max_sim_time)
        if use_smc is not None:
            self.config.use_smc = bool(use_smc)

        self.dt = self.config.dt
        self.max_sim_time = self.config.max_sim_time
        self.use_smc = self.config.use_smc
        self.drone_params = get_drone_params(self.config.drone_profile)

        self.leader = Drone(dt=self.dt, params=self.drone_params)
        self.leader.set_initial_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.followers = []
        self.winds = []
        self.topology = FormationTopology(
            num_followers=self.config.num_followers,
            spacing=self.config.formation_spacing,
        )
        self.topology.current_formation = self.config.initial_formation
        if self.config.custom_initial_offsets is not None:
            self.topology.set_custom_offsets(self.config.custom_initial_offsets)
        self.waypoints = self.config.waypoints

        self.leader_ctrl = self._build_controller()
        self._apply_controller_profile(
            self.leader_ctrl,
            gain_scale=self.config.leader_gain_scale,
            max_vel=self.config.leader_max_vel,
            max_acc=self.config.leader_max_acc,
        )

        self.follower_ctrls = []
        self._build_followers()
        self._build_winds()
        self._next_switch_idx = 0

    @staticmethod
    def _apply_controller_profile(ctrl: Controller, gain_scale: float, max_vel: float, max_acc: float) -> None:
        """按配置缩放控制参数。

        对位置环和速度环增益做等比例缩放，以统一调节响应速度，
        再配合速度和加速度限幅控制机动激进程度。
        """
        if isinstance(ctrl, BacksteppingController):
            ctrl.K0 *= gain_scale
            ctrl.K1 *= gain_scale
            ctrl.K2 *= gain_scale
        else:
            ctrl.kp_pos *= gain_scale
            ctrl.ki_pos *= gain_scale
            ctrl.kp_vel *= gain_scale
            ctrl.ki_vel *= gain_scale
            ctrl.kd_vel *= gain_scale
        ctrl.max_vel = max_vel
        ctrl.max_acc = max_acc

    def _resolved_controller_kind(self) -> str:
        kind = getattr(self.config, "controller_kind", "pid_smc")
        if kind != "pid_smc":
            return kind
        if self.config.use_backstepping:
            return "backstepping_smc"
        if self.use_smc:
            return "pid_smc"
        return "pid"

    def _build_controller(self) -> Controller:
        kind = self._resolved_controller_kind()
        common = {
            "m": self.drone_params.mass,
            "inertia": self.drone_params.inertia_matrix,
            "dt": self.dt,
        }
        if kind == "backstepping_smc":
            return BacksteppingController(**common)
        if kind == "se3_geometric":
            return GeometricSE3Controller(**common)
        if kind == "pid":
            return Controller(**common)
        ctrl = HybridAttitudeController(**common)
        ctrl.use_smc = True
        return ctrl

    def _build_followers(self) -> None:
        offsets = self.topology.get_offsets(self.config.initial_formation)
        for offset in offsets:
            follower = Drone(dt=self.dt, params=self.drone_params)
            follower.set_initial_state(self.leader.state[0:3] + offset, [0.0, 0.0, 0.0])
            self.followers.append(follower)
            ctrl = self._build_controller()
            self._apply_controller_profile(
                ctrl,
                gain_scale=self.config.follower_gain_scale,
                max_vel=self.config.follower_max_vel,
                max_acc=self.config.follower_max_acc,
            )
            self.follower_ctrls.append(ctrl)

    def _build_winds(self) -> None:
        self.leader_wind = WindField(
            steady=(0.4, 0.0, 0.0),
            turbulence_std=0.02,
            tau=self.config.wind_tau,
            seed=self.config.leader_wind_seed,
        )
        for idx in range(len(self.followers)):
            self.winds.append(
                WindField(
                    steady=(0.4, 0.0, 0.0),
                    turbulence_std=0.02,
                    tau=self.config.wind_tau,
                    seed=self.config.follower_wind_seed_start + idx,
                )
            )

    def _maybe_switch_formation(self, time_now: float) -> None:
        """按计划触发拓扑切换。"""
        schedule = self.config.formation_schedule
        if self._next_switch_idx >= len(schedule):
            return
        trigger_time, formation, transition_time = schedule[self._next_switch_idx]
        if time_now >= trigger_time:
            self.topology.switch_formation(formation, transition_time)
            self._next_switch_idx += 1

    def run(self):
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
        terminal_hold_pose: np.ndarray | None = None
        terminal_hold_steps = 0
        terminal_hold_required = max(8, int(round(0.15 / max(self.dt, 1e-6))))

        wp_radius = self.config.wp_radius
        wp_radius_final = self.config.wp_radius_final

        # 局部绑定减少循环中的属性查找开销。
        leader = self.leader
        leader_ctrl = self.leader_ctrl
        followers = self.followers
        follower_ctrls = self.follower_ctrls
        leader_wind = self.leader_wind
        winds = self.winds
        topology = self.topology
        dt = self.dt
        alpha = self.config.leader_acc_alpha

        while time_now < self.max_sim_time:
            self._maybe_switch_formation(time_now)

            if terminal_hold_pose is not None:
                target_wp = terminal_hold_pose
            elif not finished:
                target_wp = self.waypoints[current_wp_idx]
            else:
                target_wp = self.waypoints[-1]

            wind_leader = leader_wind.sample(dt)
            leader_pos, leader_vel, _, _ = leader.get_state()
            leader_u = leader_ctrl.compute_control(leader.state, target_wp)
            leader.update_state(leader_u, wind=wind_leader)
            leader_pos_new, leader_vel_new, _, _ = leader.get_state()

            if not finished:
                dist_to_wp = np.linalg.norm(target_wp - leader_pos_new)
                radius = wp_radius_final if current_wp_idx == len(self.waypoints) - 1 else wp_radius
                if current_wp_idx == len(self.waypoints) - 1:
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
                    if current_wp_idx >= len(self.waypoints):
                        finished = True

            leader_acc = (leader_vel_new - leader_vel) / dt
            leader_acc_filt = alpha * leader_acc + (1.0 - alpha) * leader_acc_filt
            offsets = topology.get_current_offsets(time_now)

            for i, follower in enumerate(followers):
                wind_follower = winds[i].sample(dt)
                target_pos = leader_pos_new + offsets[i]
                follower_u = follower_ctrls[i].compute_control(
                    follower.state,
                    target_pos,
                    target_vel=leader_vel_new,
                    target_acc=leader_acc_filt,
                )
                follower.update_state(follower_u, wind=wind_follower)
                follower_pos = follower.get_state()[0]
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

        return {
            "time": history_time[valid],
            "leader": history_leader[valid, :],
            "followers": [history_followers[i, valid, :] for i in range(follower_count)],
            "targets": [target_positions[i, valid, :] for i in range(follower_count)],
            "error_vectors": [error_vectors[i, valid, :] for i in range(follower_count)],
            "errors": errors,
            "metrics": metrics,
            "completed_waypoint_count": len(self.waypoints) if finished else current_wp_idx,
            "waypoints": np.array(self.waypoints, dtype=float),
            "waypoint_events": waypoint_events,
        }
