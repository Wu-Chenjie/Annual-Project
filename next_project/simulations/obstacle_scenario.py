"""闅滅鐗╁満鏅豢鐪熺紪鎺掋€?
鐢ㄩ€?----
鍦?FormationSimulation 鍩虹涓婂紩鍏ラ殰纰嶇墿鎰熺煡銆佽矾寰勮鍒掍笌閬跨妫€娴嬶紝
鏀寔绂荤嚎瑙勫垝锛堝叏灞€鑸偣锛変笌鍦ㄧ嚎瑙勫垝锛堜紶鎰熷櫒 + 婊戝姩绐楀彛锛変袱鏉¤矾绾裤€?
鍘熺悊
----
1) 闅滅鐗╁満 鈫?OccupancyGrid锛堝惈鑶ㄨ儉灞傦級 鈫?Planner 鐢熸垚鑸偣銆?2) 鍦ㄧ嚎妯″紡涓嬶紝WindowReplanner 鎸?interval 鍛ㄦ湡鎬ф洿鏂拌埅鐐广€?3) 姣忔杩涜鍦伴潰鐪熷€肩鎾炴娴嬶紝璁板綍浣嗕笉涓柇浠跨湡銆?
"""

from __future__ import annotations

import time

import numpy as np

from .formation_simulation import FormationSimulation, SimulationConfig
from core.obstacles import ObstacleField, SDFAwareGrid
from core.map_loader import load_from_json, load_from_npz
from core.sensors import RangeSensor6
from core.artificial_potential_field import ImprovedArtificialPotentialField
from core.fault_detector import FaultDetector
from core.formation_safety import (
    FormationSafetyConfig,
    deconflict_follower_target,
    downwash_zone,
    follower_safety_correction,
    is_in_downwash_zone,
    min_inter_drone_distance,
    nominal_target_ready_for_recovery,
)
from core.formation_clearance import FormationClearancePolicy
from core.formation_adaptation import FormationAdaptationPolicy
from core.topology import FormationTopology
from core.planning import (
    AStar, TurnConstrainedAStar, HybridAStar, Dijkstra, RRTStar,
    InformedRRTStar, DStarLite, WindowReplanner, RiskAdaptiveReplanInterval,
    Planner, CostAwareGrid,
    VisibilityGraph, GNNPlanner, DualModeScheduler, FormationAPF,
    TrajectoryOptimizer,
)
from core.planning.firi import FIRIRefiner


def make_planner(kind: str, **kw) -> Planner:
    """鎸夊悕绉版瀯閫犺鍒掑櫒瀹炰緥銆"""
    if kind == "astar":
        return AStar()
    if kind == "turn_constrained_astar":
        return TurnConstrainedAStar(max_turn_rad=kw.get("max_turn_rad", 0.7854))
    if kind == "hybrid_astar":
        return HybridAStar(
            v_max=kw.get("v_max", 2.0),
            v_z_max=kw.get("v_z_max", 1.0),
            omega_max=kw.get("omega_max", 1.0472),
            dt_primitive=kw.get("dt_primitive", 0.5),
            max_iter=kw.get("max_iter", 10000),
            analytic_expand_interval=kw.get("analytic_expand_interval", 50),
        )
    if kind == "dijkstra":
        return Dijkstra()
    if kind == "rrt_star":
        return RRTStar(
            max_iter=kw.get("max_iter", 4000),
            rewire_radius=kw.get("rewire_radius", 1.5),
            smooth_method=kw.get("smooth_method", "bspline"),
        )
    if kind == "informed_rrt_star":
        return InformedRRTStar(
            max_iter=kw.get("max_iter", 4000),
            rewire_radius=kw.get("rewire_radius", 1.5),
            smooth_method=kw.get("smooth_method", "bspline"),
        )
    raise ValueError(f"鏈煡瑙勫垝鍣? {kind}")


class ObstacleScenarioSimulation(FormationSimulation):
    """带障碍物与在线重规划的编队仿真器。"""

    def __init__(self, config: SimulationConfig):
        first_wp = config.waypoints[0] if config.waypoints else np.zeros(3)
        self.planned_trajectory = None
        self.planning_events: list[dict] = []
        super().__init__(config=config)
        arm_length = 0.2  # Drone 默认臂长
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

    def _build_apf(self) -> ImprovedArtificialPotentialField:
        cfg = self.config
        profile = getattr(cfg, "apf_paper1_profile", "off")
        params = {
            "k_rep": 0.8,
            "r_rep": 2.5,
            "n_decay": 2,
            "k_inter": 0.2,
            "s_inter": 2.0,
            "mu_escape": 0.5,
            "max_acc": 8.0,
            "adaptive_n_decay": False,
            "k_comm": 0.0,
            "comm_range": cfg.apf_comm_range,
        }
        if profile == "conservative":
            params.update({
                "adaptive_n_decay": True,
                "k_comm": 0.15,
                "mu_escape": 0.35,
            })
        elif profile == "aggressive":
            params.update({
                "adaptive_n_decay": True,
                "k_comm": 0.30,
                "mu_escape": 0.60,
                "k_inter": 0.25,
            })
        if getattr(cfg, "apf_dev_override", False):
            params["adaptive_n_decay"] = bool(cfg.apf_adaptive_n_decay)
            params["k_comm"] = 0.3 if getattr(cfg, "apf_comm_constraint", False) else 0.0
            params["mu_escape"] = 0.5 if getattr(cfg, "apf_rotational_escape", False) else 0.0
        return ImprovedArtificialPotentialField(**params)

    def _build_fault_detector(self) -> FaultDetector | None:
        cfg = self.config
        if not getattr(cfg, "fault_detection_enabled", False):
            return None
        return FaultDetector(
            max_acc=cfg.fault_detector_max_acc,
            pos_dev_threshold=cfg.fault_detector_pos_dev,
            saturate_steps=cfg.fault_detector_saturate_steps,
            dt=self.dt,
        )

    def _build_formation_apf(self) -> FormationAPF | None:
        cfg = self.config
        if not getattr(cfg, "apf_formation_centroid", False):
            return None
        return FormationAPF(
            k_rep=self.apf.k_rep,
            r_rep=self.apf.r_rep,
            alpha=cfg.apf_centroid_alpha,
            beta=cfg.apf_centroid_beta,
        )

    def _setup_obstacles(self) -> None:
        """鍔犺浇闅滅鐗╁満骞朵綋绱犲寲銆"""
        cfg = self.config
        if cfg.map_file is not None:
            filepath = cfg.map_file
            if filepath.endswith(".json"):
                self.obstacles, self._map_bounds = load_from_json(filepath)
            elif filepath.endswith(".npz"):
                grid = load_from_npz(filepath)
                self.obstacles = ObstacleField()
                self._map_bounds = np.array([
                    grid.origin,
                    grid.origin + np.array(grid.shape) * grid.resolution,
                ])
                self.grid = grid
            else:
                raise ValueError(f"涓嶆敮鎸佺殑鍦板浘鏍煎紡: {filepath}")
        elif cfg.obstacle_field is not None:
            self.obstacles = cfg.obstacle_field
            self._map_bounds = np.array([[-10, -10, 0], [30, 30, 10]], dtype=float)
        else:
            self.obstacles = ObstacleField()
            self._map_bounds = np.array([[-10, -10, 0], [30, 30, 10]], dtype=float)

        # 浣撶礌鍖栵紙濡傚皻鏈粠 NPZ 鍔犺浇锛?
        if not hasattr(self, "grid") or self.grid is None:
            self.grid = self.obstacles.to_voxel_grid(self._map_bounds, cfg.planner_resolution)
            self.grid = self.grid.inflate(self._inflate_margin_xyz())

        # SDF 鎰熺煡鍖呰锛氱簿鍒よ杽闅滅鐗╋紙鍗婂緞 < 鏍呮牸鍒嗚鲸鐜囷級锛岄伩鍏嶄綋绱犲寲涓㈠け
        if getattr(cfg, "planner_sdf_aware", False):
            clearance = max(float(cfg.safety_margin), float(getattr(self, "_collision_margin", 0.0)))
            self.grid = SDFAwareGrid(
                self.grid,
                self.obstacles,
                clearance=clearance,
            )

        # ESDF 杞唬浠峰寘瑁咃細鎵€鏈夎鍒掑櫒鑷姩鍊惧悜杩滅闅滅鐗╃殑璺緞
        if getattr(cfg, "planner_esdf_aware", True):
            self.grid = CostAwareGrid(
                self.grid,
                weight=2.0,
                scale=1.5,
                cap_distance=4.0,
            )
        self._apply_planning_z_bounds()

    def _apply_planning_z_bounds(self) -> None:
        """将规划搜索限制在配置的高度层内，避免室内场景绕到天花板或墙体上方。"""
        z_bounds = getattr(self.config, "planner_z_bounds", None)
        if z_bounds is None:
            return
        z_min, z_max = float(z_bounds[0]), float(z_bounds[1])
        data = self.grid.data
        for iz in range(self.grid.shape[2]):
            z = float(self.grid.origin[2] + iz * self.grid.resolution)
            if z < z_min - 1e-9 or z > z_max + 1e-9:
                data[:, :, iz] = 1

    def _channel_width_from_sensor(self, sensor_reading: np.ndarray | None) -> tuple[float, float, float] | None:
        """将六向测距读数折算为局部三轴通道宽度。"""
        if sensor_reading is None or len(sensor_reading) < 6:
            return None
        reading = np.asarray(sensor_reading, dtype=float)
        return (
            float(reading[2] + reading[3]),
            float(reading[0] + reading[1]),
            float(reading[4] + reading[5]),
        )

    def _formation_clearance_base(self, clearance: float | None = None) -> float:
        if clearance is not None:
            return float(clearance)
        return max(float(self.config.safety_margin), float(getattr(self, "_collision_margin", 0.0))) + float(
            self.config.plan_clearance_extra
        )

    def _formation_clearance_enabled(self) -> bool:
        return bool(
            getattr(self.config, "planner_use_formation_envelope", False)
            or getattr(self.config, "formation_safety_enabled", False)
        )

    def _active_formation_name(self) -> str:
        if getattr(self.topology, "is_switching", False) and getattr(self.topology, "_target_formation", ""):
            return str(self.topology._target_formation)
        return str(getattr(self.topology, "current_formation", self.config.initial_formation))

    def _make_formation_clearance_policy(
        self,
        clearance: float | None = None,
        *,
        formation_aware: bool | None = None,
    ) -> FormationClearancePolicy:
        formation = self._active_formation_name()
        try:
            offsets = self.topology.get_offsets(formation)
        except Exception:
            offsets = self.topology.get_offsets(self.config.initial_formation)
        sample_spacing = max(min(float(self.config.planner_resolution) * 0.5, 0.10), 0.05)
        return FormationClearancePolicy(
            signed_distance=lambda p: float(self.obstacles.signed_distance(np.asarray(p, dtype=float))),
            follower_offsets=offsets,
            base_clearance=self._formation_clearance_base(clearance),
            sample_spacing=sample_spacing,
            formation_aware=self._formation_clearance_enabled() if formation_aware is None else bool(formation_aware),
        )

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

    def _inflate_margin_xyz(self) -> float | tuple[float, float, float]:
        """返回规划栅格膨胀半径。

        标量接口继续服务于传统单机/固定裕度场景；启用编队包络时，使用三轴半径
        让横向通道宽度不再被纵向编队长度错误放大。
        """
        if not getattr(self.config, "planner_use_formation_envelope", False):
            return float(self.config.safety_margin)
        lateral, longitudinal, vertical = self.topology.envelope_per_axis()
        margin = float(self.config.safety_margin)
        return (
            lateral + margin,
            longitudinal + margin,
            vertical + margin,
        )

    def _rebuild_planning_grid(self) -> None:
        """按当前队形重新生成规划网格，避免队形切换后继续沿用旧膨胀层。"""
        if self._map_bounds is None or len(self._map_bounds) == 0:
            return
        base_grid = self.obstacles.to_voxel_grid(self._map_bounds, self.config.planner_resolution)
        plan_grid = base_grid.inflate(self._inflate_margin_xyz())
        if getattr(self.config, "planner_sdf_aware", False):
            clearance = max(float(self.config.safety_margin), float(getattr(self, "_collision_margin", 0.0)))
            plan_grid = SDFAwareGrid(
                plan_grid,
                self.obstacles,
                clearance=clearance,
            )
        if getattr(self.config, "planner_esdf_aware", True):
            plan_grid = CostAwareGrid(
                plan_grid,
                weight=2.0,
                scale=1.5,
                cap_distance=4.0,
            )
        self.grid = plan_grid
        self._apply_planning_z_bounds()
        if hasattr(self, "replanner") and self.replanner is not None:
            self.replanner.grid = self.grid
            self.replanner._static_occupied = (np.asarray(self.grid.data) >= 1).copy()
            self.replanner._sensor_occupied = np.zeros_like(self.grid.data, dtype=bool)
            self.replanner._sensor_ttl = np.zeros_like(self.grid.data, dtype=np.int16)
            self.replanner._sensor_clear_hits = np.zeros_like(self.grid.data, dtype=np.int16)
            self.replanner._changed_cells_since_last = []
            self.replanner._current_path = None
            self.replanner._global_ref_path = None
            self.replanner.incremental_planner = None

    def _setup_planning(self) -> None:
        """鍒濆鍖栬鍒掑櫒涓庡湪绾跨粍浠讹紙鍚鏂?-3 鍒嗗眰鏋舵瀯锛夈€"""
        cfg = self.config
        self.planner = make_planner(
            cfg.planner_kind,
            max_iter=getattr(cfg, "planner_max_iter", 4000),
            rewire_radius=getattr(cfg, "planner_rewire_radius", 1.5),
            smooth_method="bspline",
        )
        self.firi_refiner = FIRIRefiner(
            self.obstacles,
            min_clearance=cfg.safety_margin + cfg.plan_clearance_extra,
        )
        self.trajectory_optimizer = TrajectoryOptimizer(
            nominal_speed=getattr(cfg, "trajectory_optimizer_nominal_speed", 1.0),
            sample_dt=getattr(cfg, "trajectory_optimizer_sample_dt", 0.2),
        )
        self._planning_waypoints = self._sanitize_waypoints(cfg.waypoints)
        self._task_waypoints = [wp.copy() for wp in cfg.waypoints]
        self.config.waypoints = [wp.copy() for wp in self._planning_waypoints]
        self.waypoints = self.config.waypoints
        self.planned_path: np.ndarray | None = None
        self._maybe_preplan_formation_adaptation()

        if cfg.planner_mode == "offline":
            self.planned_path = self._plan_offline()
            if self.planned_path is not None and len(self.planned_path) > 0:
                self.config.waypoints = [wp for wp in self.planned_path]
                self.waypoints = self.config.waypoints
        else:
            self.planned_path = self._plan_offline()
            self.config.waypoints = [wp.copy() for wp in self._planning_waypoints]
            self.waypoints = self.config.waypoints

            # ---- 鍏ㄥ眬鍙傝€冨眰锛欼nformed RRT* ----
            global_planner = None
            try:
                global_planner = InformedRRTStar(
                    max_iter=4000,
                    rewire_radius=1.5,
                    smooth_method="bspline",
                )
            except Exception:
                pass

            # 澧為噺閫€鍖栧眰锛欴* Lite锛堝欢杩熷垵濮嬪寲锛岄渶瑕?start/goal锛?
            incremental_planner = None

            # ---- 璁烘枃2: Danger 妯″紡缁勪欢锛圙NN + 鎯版€у彲瑙佸浘 + 鍙屾ā寮忚皟搴︼級 ----
            danger_planner = None
            dual_mode = None
            if getattr(cfg, "danger_mode_enabled", False):
                try:
                    danger_planner = GNNPlanner(
                        A=10.0, B=1.0, D=1.0,
                        gamma=cfg.gnn_gamma,
                        alpha=cfg.gnn_alpha,
                        beta=cfg.gnn_beta,
                        V=cfg.gnn_V,
                        E=cfg.gnn_E,
                    )
                    # 鎯版€ф瀯寤猴細棣栨 Danger replan 鏃舵瀯寤哄彲瑙佸浘锛岄伩鍏?init 闃诲
                    danger_planner._lazy_obstacles = self.obstacles
                    danger_planner._lazy_angular_res = cfg.gnn_angular_res
                    danger_planner._lazy_buffer_zone = cfg.gnn_buffer_zone
                    danger_planner._lazy_visible_range = cfg.planner_horizon * 4
                    danger_planner._cached_vis_graph = None
                    dual_mode = DualModeScheduler(
                        sensor_danger_threshold=cfg.sensor_danger_threshold,
                        sensor_safe_threshold=cfg.sensor_safe_threshold,
                        sdf_danger_threshold=cfg.sdf_danger_threshold,
                    )
                except Exception:
                    pass

            # ---- 璁烘枃3: 椋庨櫓椹卞姩鑷€傚簲閲嶈鍒掗棿闅?----
            adaptive_interval = None
            if getattr(cfg, "replan_adaptive_interval", False):
                adaptive_interval = RiskAdaptiveReplanInterval(
                    base_interval=cfg.planner_replan_interval,
                    min_interval=cfg.replan_interval_min,
                    max_interval=cfg.replan_interval_max,
                )

            self.replanner = WindowReplanner(
                self.planner,
                self.grid,
                interval=cfg.planner_replan_interval,
                horizon=cfg.planner_horizon,
                deviation_metric="hausdorff",
                global_planner=global_planner,
                incremental_planner=incremental_planner,
                local_fail_threshold=3,
                danger_planner=danger_planner,
                dual_mode=dual_mode,
                adaptive_interval=adaptive_interval,
                obstacle_field=self.obstacles,
            )
            self.replanner.path_refiner = self.firi_refiner

            if cfg.sensor_enabled:
                self.sensor = RangeSensor6(
                    max_range=cfg.sensor_max_range,
                    noise_std=cfg.sensor_noise_std,
                    seed=cfg.leader_wind_seed,
                )
            else:
                self.sensor = None

    def _sanitize_waypoints(self, waypoints: list[np.ndarray]) -> list[np.ndarray]:
        """灏嗚鑶ㄨ儉鏍呮牸瑕嗙洊鎴栬惤鍏ラ殰纰嶇殑鑸偣淇鍒版渶杩戝彲瑙勫垝浣嶇疆銆"""
        if not waypoints:
            return []

        safe_waypoints: list[np.ndarray] = []
        count = len(waypoints)
        for i, waypoint in enumerate(waypoints):
            prefer = None
            if i + 1 < count:
                prefer = np.asarray(waypoints[i + 1], dtype=float) - np.asarray(waypoint, dtype=float)
            elif i > 0:
                prefer = np.asarray(waypoint, dtype=float) - np.asarray(waypoints[i - 1], dtype=float)
            safe_waypoints.append(self._project_to_planning_free(waypoint, prefer=prefer))
        return safe_waypoints

    def _project_to_planning_free(
        self,
        point: np.ndarray,
        prefer: np.ndarray | None = None,
        min_clearance: float | None = None,
        max_radius_m: float | None = None,
    ) -> np.ndarray:
        """灏嗙偣鎶曞奖鍒版渶杩戠殑瑙勫垝鑷敱鐐癸紝瑙ｅ喅鑶ㄨ儉鍚庤埅鐐硅瑕嗙洊鐨勯棶棰樸€"""
        point = np.asarray(point, dtype=float)
        min_clearance = float(
            max(self.config.safety_margin, getattr(self, "_collision_margin", 0.0))
            + self.config.plan_clearance_extra
            if min_clearance is None else min_clearance
        )
        if max_radius_m is None:
            max_radius_m = max(6.0, self._inflate_r() + 2.0)

        point_sd = float(self.obstacles.signed_distance(point))
        point_idx = self.grid.world_to_index(point)
        if (not self.grid.is_occupied(point_idx)) and point_sd >= min_clearance - 1e-8:
            return point.copy()

        prefer_dir = None
        if prefer is not None:
            prefer = np.asarray(prefer, dtype=float)
            prefer_norm = float(np.linalg.norm(prefer))
            if prefer_norm > 1e-9:
                prefer_dir = prefer / prefer_norm

        max_radius_vox = max(1, int(np.ceil(max_radius_m / self.grid.resolution)))
        shape = np.asarray(self.grid.shape, dtype=int)
        best_point = None
        best_score = float("inf")

        for radius in range(max_radius_vox + 1):
            i0_min = max(0, point_idx[0] - radius)
            i0_max = min(shape[0] - 1, point_idx[0] + radius)
            i1_min = max(0, point_idx[1] - radius)
            i1_max = min(shape[1] - 1, point_idx[1] + radius)
            i2_min = max(0, point_idx[2] - radius)
            i2_max = min(shape[2] - 1, point_idx[2] + radius)

            for ix in range(i0_min, i0_max + 1):
                for iy in range(i1_min, i1_max + 1):
                    for iz in range(i2_min, i2_max + 1):
                        if max(abs(ix - point_idx[0]), abs(iy - point_idx[1]), abs(iz - point_idx[2])) != radius:
                            continue
                        idx = (ix, iy, iz)
                        if self.grid.is_occupied(idx):
                            continue
                        candidate = self.grid.index_to_world(idx)
                        sd = float(self.obstacles.signed_distance(candidate))
                        if sd < min_clearance - 1e-8:
                            continue

                        delta = candidate - point
                        score = float(np.linalg.norm(delta))
                        if prefer_dir is not None:
                            delta_norm = float(np.linalg.norm(delta))
                            if delta_norm > 1e-9:
                                direction = delta / delta_norm
                                score += 0.25 * (1.0 - float(np.dot(direction, prefer_dir)))
                        score += 0.05 * abs(float(candidate[2] - point[2]))

                        if score < best_score:
                            best_score = score
                            best_point = candidate

            if best_point is not None:
                return np.asarray(best_point, dtype=float)

        fallback = self.firi_refiner._push_out_if_needed(point)
        fallback_idx = self.grid.world_to_index(fallback)
        if not self.grid.is_occupied(fallback_idx):
            return fallback
        return point.copy()

    def _inflate_r(self) -> float:
        """璁＄畻鑶ㄨ儉鍗婂緞 = 缂栭槦鍖呯粶锛堝惈鑷傞暱锛?+ 瀹夊叏瑁曞害銆?
        envelope_radius 宸插寘鍚?arm_length锛屾澶勪笉鍐嶉噸澶嶅彔鍔犮€?        """
        if not getattr(self.config, "planner_use_formation_envelope", False):
            return self.config.safety_margin
        lateral, _, vertical = self.topology.envelope_per_axis()
        return max(lateral, vertical) + self.config.safety_margin

    def _planned_segment_for_task(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """从离线安全参考路径中截取当前位置到当前任务航点的局部参考。"""
        if self.planned_path is None or len(self.planned_path) < 2:
            return None
        ref = np.asarray(self.planned_path, dtype=float)
        start_i = int(np.argmin(np.linalg.norm(ref - np.asarray(start, dtype=float), axis=1)))
        goal_i = int(np.argmin(np.linalg.norm(ref - np.asarray(goal, dtype=float), axis=1)))
        if goal_i <= start_i:
            segment = np.vstack([start, goal])
        else:
            segment = np.vstack([start, ref[start_i + 1:goal_i + 1]])
            if np.linalg.norm(segment[-1] - goal) > max(self.config.wp_radius * 0.5, self.grid.resolution):
                segment = np.vstack([segment, goal])
        return np.asarray(segment, dtype=float)

    def _extend_online_path_to_task(self, path: np.ndarray, task_goal: np.ndarray) -> np.ndarray:
        """将在线窗口子路径拼接到当前任务航点，避免到窗口末端后停止。"""
        path = np.asarray(path, dtype=float)
        task_goal = np.asarray(task_goal, dtype=float)
        if len(path) == 0:
            return np.array([task_goal.copy()], dtype=float)
        if np.linalg.norm(path[-1] - task_goal) <= max(self.config.wp_radius, self.grid.resolution):
            return path

        tail = self._planned_segment_for_task(path[-1], task_goal)
        if tail is not None and len(tail) >= 2 and self._segment_is_safe(tail, self._collision_margin):
            if np.linalg.norm(tail[0] - path[-1]) < 1e-6:
                return np.vstack([path, tail[1:]])
            return np.vstack([path, tail])

        direct = np.vstack([path[-1], task_goal])
        if self._segment_is_safe(direct, self._collision_margin):
            return np.vstack([path, task_goal])
        return path

    def _accept_online_path(
        self,
        path: np.ndarray,
        leader_pos: np.ndarray,
        task_goal: np.ndarray,
        time_now: float,
    ) -> np.ndarray | None:
        """在线路径接收门禁：连续 clearance 不合格时重试，而不是继续执行不安全路径。"""
        path = np.asarray(path, dtype=float)
        leader_pos = np.asarray(leader_pos, dtype=float)
        task_goal = np.asarray(task_goal, dtype=float)
        min_clearance = max(
            float(self._collision_margin) + 0.03,
            float(self.config.safety_margin) + float(self.config.plan_clearance_extra),
        )

        if len(path) == 0:
            return None
        if np.linalg.norm(path[0] - leader_pos) > self.grid.resolution * 1.5:
            path = np.vstack([leader_pos, path])
        path = self._extend_online_path_to_task(path, task_goal)

        try:
            refined_path = self.firi_refiner.refine(path, seeds=path)
            if self._segment_is_safe(refined_path, min_clearance):
                return np.asarray(refined_path, dtype=float)
        except Exception:
            pass

        if self._segment_is_safe(path, min_clearance):
            return path

        fallback = self._plan_segment_fallback(leader_pos, task_goal, min_clearance)
        if fallback is not None:
            fallback = self._extend_online_path_to_task(fallback, task_goal)
            if self._segment_is_safe(fallback, min_clearance):
                self.replan_events.append({
                    "t": float(time_now),
                    "mode": "clearance_fallback",
                    "reason": "continuous_clearance_blocked",
                })
                return np.asarray(fallback, dtype=float)

        self.replan_events.append({
            "t": float(time_now),
            "mode": "clearance_blocked",
            "reason": "no_continuous_clearance_path",
        })
        return None

    def _path_segment_clearance(self, path: np.ndarray, min_clearance: float, spacing: float | None = None) -> float:
        """返回路径线段采样得到的最小 SDF 间隙。"""
        path = np.asarray(path, dtype=float)
        if self._formation_clearance_enabled():
            policy = self._make_formation_clearance_policy(min_clearance)
            evaluation = policy.evaluate_path(
                path,
                base_clearance=float(min_clearance),
                sample_spacing=spacing,
            )
            return float(evaluation.min_formation_signed_distance)
        if len(path) == 0:
            return float("-inf")
        if len(path) == 1:
            return float(self.obstacles.signed_distance(path[0]))
        sample_spacing = float(spacing or max(min(self.grid.resolution * 0.5, 0.10), 0.08))
        worst = float("inf")
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            dist = float(np.linalg.norm(b - a))
            n = max(1, int(np.ceil(dist / sample_spacing)))
            for j in range(n + 1):
                point = a + (b - a) * (j / n)
                worst = min(worst, float(self.obstacles.signed_distance(point)))
                if worst < 0.0:
                    return worst
        return worst

    def _segment_is_safe(self, path: np.ndarray, min_clearance: float) -> bool:
        return self._path_segment_clearance(path, min_clearance) >= min_clearance - 1e-6

    def _enforce_path_clearance(self, path: np.ndarray, min_clearance: float,
                                max_iter: int = 60, step: float = 0.10) -> np.ndarray:
        """SDF 姊害涓嬮檷淇璺緞鑸偣锛岀‘淇濈紪闃熷叏浣撳埌闅滅鐗╄窛绂昏揪鏍囥€?
        瀵规瘡涓埅鐐规鏌ラ鑸満涓庢墍鏈変粠鏈轰綅缃槸鍚︽弧瓒?min_clearance锛?        涓嶆弧瓒冲垯娌?SDF 姊害鎺ㄧ銆傞殢鍚庡湪绾挎涓偣閲囨牱鏍℃牳骞舵彃鍏ラ澶栬埅鐐广€?        """
        path = path.copy()
        eps = 0.02
        offsets = self.topology.get_offsets(self.config.initial_formation)

        # 閫愯埅鐐规搴︿慨姝?
        for idx in range(len(path)):
            wp = path[idx]
            for _ in range(max_iter):
                min_sd = self.obstacles.signed_distance(wp)
                worst_pos = wp
                for off in offsets:
                    check_pos = wp + off
                    sd = self.obstacles.signed_distance(check_pos)
                    if sd < min_sd:
                        min_sd = sd
                        worst_pos = check_pos
                if min_sd >= min_clearance:
                    break
                grad = np.array([
                    self.obstacles.signed_distance(worst_pos + [eps, 0, 0])
                    - self.obstacles.signed_distance(worst_pos - [eps, 0, 0]),
                    self.obstacles.signed_distance(worst_pos + [0, eps, 0])
                    - self.obstacles.signed_distance(worst_pos - [0, eps, 0]),
                    self.obstacles.signed_distance(worst_pos + [0, 0, eps])
                    - self.obstacles.signed_distance(worst_pos - [0, 0, eps]),
                ], dtype=float) / (2.0 * eps)
                grad_norm = float(np.linalg.norm(grad))
                if grad_norm < 1e-10:
                    break
                wp = wp + grad / grad_norm * step
            path[idx] = wp

        # 绾挎閲囨牱鏍℃牳锛氬湪鐩搁偦鑸偣闂撮噰鏍峰苟鎻掑叆瀹夊叏淇鐐?
        dense_path: list[np.ndarray] = [path[0]]
        n_samples = 8
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            worst_sd = min_clearance
            worst_t = 0.0
            for j in range(1, n_samples):
                t = j / n_samples
                sp = a + (b - a) * t
                min_sd_sp = self.obstacles.signed_distance(sp)
                for off in offsets:
                    sd = self.obstacles.signed_distance(sp + off)
                    if sd < min_sd_sp:
                        min_sd_sp = sd
                if min_sd_sp < worst_sd:
                    worst_sd = min_sd_sp
                    worst_t = t
            if worst_sd < min_clearance:
                # 鍦ㄧ嚎娈垫渶宸偣鎻掑叆棰濆鑸偣锛堟帹鍚戝畨鍏ㄤ晶锛?
                sp = a + (b - a) * worst_t
                sp = self._enforce_path_clearance(np.array([sp]), min_clearance, max_iter=40, step=0.08)[0]
                dense_path.append(sp)
            dense_path.append(b)

        return np.array(dense_path, dtype=float)

    def _plan_offline(self) -> np.ndarray | None:
        """绂荤嚎鍏ㄥ眬瑙勫垝銆?
        鍘熺悊
        ----
        1) A* 鍦ㄨ啫鑳€鏍呮牸涓婃悳绱㈣矾寰勶紙鏍呮牸瀵硅杽闅滅鐗╀笉鏁忔劅锛夈€?        2) Catmull-Rom 骞虫粦銆?        3) SDF 鏍℃牳鎺ㄧ锛氬闈犺繎闅滅鐗╃殑鑸偣娌挎搴︽帹鑷冲畨鍏ㄨ窛绂汇€?        4) 绾挎閲囨牱鎻掑€硷細鐩搁偦鑸偣闂磋嫢瀛樺湪涓嶅畨鍏ㄥ尯鍩熷垯鎻掑叆棰濆鑸偣銆?        """
        waypoints = self._planning_waypoints
        if len(waypoints) < 2:
            return None

        grid = self.grid
        min_clearance = max(self.config.safety_margin, self._collision_margin) + self.config.plan_clearance_extra

        full_path: list[np.ndarray] = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            goal = waypoints[i + 1]
            segment_started = time.perf_counter()
            fallback_reason = None
            try:
                segment = self.planner.plan(start, goal, grid)
            except Exception:
                fallback_reason = "primary_planner_failed"
                fallback = self._plan_segment_fallback(start, goal, min_clearance)
                if fallback is None:
                    self.planning_events.append({
                        "t": 0.0,
                        "phase": "offline_segment",
                        "planner": str(getattr(self.config, "planner_kind", "")),
                        "segment_index": int(i),
                        "wall_time_s": float(time.perf_counter() - segment_started),
                        "point_count": 0,
                        "accepted": False,
                        "fallback_reason": fallback_reason,
                    })
                    continue
                segment = fallback

            if not self._segment_is_safe(segment, min_clearance):
                fallback = self._plan_segment_fallback(start, goal, min_clearance)
                if fallback is not None:
                    fallback_reason = "clearance_fallback"
                    segment = fallback
                elif not self._segment_is_safe(segment, 0.0):
                    self.planning_events.append({
                        "t": 0.0,
                        "phase": "offline_segment",
                        "planner": str(getattr(self.config, "planner_kind", "")),
                        "segment_index": int(i),
                        "wall_time_s": float(time.perf_counter() - segment_started),
                        "point_count": int(len(segment)) if segment is not None else 0,
                        "accepted": False,
                        "fallback_reason": "unsafe_segment",
                    })
                    continue

            try:
                smoothed = self.planner.smooth(segment)
            except Exception:
                smoothed = np.asarray(segment, dtype=float)
            if not self._segment_is_safe(smoothed, 0.0):
                smoothed = np.asarray(segment, dtype=float)
            if not self._segment_is_safe(smoothed, min_clearance):
                smoothed = self._enforce_path_clearance(smoothed, min_clearance)
            if not self._segment_is_safe(smoothed, 0.0):
                self.planning_events.append({
                    "t": 0.0,
                    "phase": "offline_segment",
                    "planner": str(getattr(self.config, "planner_kind", "")),
                    "segment_index": int(i),
                    "wall_time_s": float(time.perf_counter() - segment_started),
                    "point_count": int(len(smoothed)) if smoothed is not None else 0,
                    "accepted": False,
                    "fallback_reason": "unsafe_smoothed_segment",
                })
                continue
            # 鎺ㄨ繙鍚庡姞瀵嗭細SDF 鎺ㄧ浼氬澶ч棿闅欙紝寮ч暱閲嶉噰鏍疯ˉ鍥炲瘑搴?
            if min_clearance > self.config.safety_margin + 0.25:
                target_spacing = max(grid.resolution * 1.5, 0.6)
                smoothed = self._resample_path(smoothed, target_spacing)
            refined = self.firi_refiner.refine(smoothed, seeds=segment)
            if self._segment_is_safe(refined, 0.0):
                smoothed = refined
            smoothed = self._resample_path(smoothed, max(grid.resolution * 2.0, 0.5))

            if len(full_path) > 0:
                first = smoothed[0]
                last = full_path[-1]
                if np.linalg.norm(first - last) < 1e-4:
                    smoothed = smoothed[1:]
            self.planning_events.append({
                "t": 0.0,
                "phase": "offline_segment",
                "planner": str(getattr(self.config, "planner_kind", "")),
                "segment_index": int(i),
                "wall_time_s": float(time.perf_counter() - segment_started),
                "point_count": int(len(smoothed)),
                "accepted": True,
                "fallback_reason": fallback_reason,
            })
            full_path.extend(smoothed.tolist())
        planned = np.array(full_path, dtype=float)
        if len(planned) == 0:
            return planned
        if getattr(self.config, "trajectory_optimizer_enabled", False):
            self.planned_trajectory = self.trajectory_optimizer.optimize(
                planned,
                method=getattr(self.config, "trajectory_optimizer_method", "moving_average"),
                clearance_checker=lambda candidate: self._segment_is_safe(candidate, min_clearance),
                fallback_to_raw=True,
            )
            return np.asarray(self.planned_trajectory.positions, dtype=float)
        self.planned_trajectory = None
        return planned

    def _plan_segment_fallback(self, start: np.ndarray, goal: np.ndarray, min_clearance: float) -> np.ndarray | None:
        """段规划失败时使用低约束 A* 重试，禁止直接用直线穿障碍兜底。"""
        try:
            fallback_grid = self.obstacles.to_voxel_grid(self._map_bounds, self.config.planner_resolution)
            fallback_grid = fallback_grid.inflate(max(self.config.safety_margin, min_clearance))
            if getattr(self.config, "planner_sdf_aware", False):
                fallback_grid = SDFAwareGrid(
                    fallback_grid,
                    self.obstacles,
                    clearance=max(self.config.safety_margin, self._collision_margin, min_clearance),
                )
            path = AStar().plan(start, goal, fallback_grid)
            return np.asarray(path, dtype=float)
        except Exception:
            direct = np.array([start, goal], dtype=float)
            if self._segment_is_safe(direct, min_clearance):
                return direct
        return None

    @staticmethod
    def _resample_path(path: np.ndarray, spacing: float) -> np.ndarray:
        """寮ч暱閲嶉噰鏍峰埌鐩爣闂磋窛锛堝姞瀵嗘垨闄嶉噰鏍凤級銆"""
        if len(path) < 2:
            return path
        path = np.asarray(path, dtype=float)
        diffs = np.diff(path, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum_len = np.concatenate([[0.0], np.cumsum(seg_lens)])
        total = cum_len[-1]
        if total < spacing:
            return path
        n = max(2, int(np.ceil(total / spacing)))
        sample_lens = np.linspace(0, total, n)
        result = np.zeros((n, 3))
        for d in range(3):
            result[:, d] = np.interp(sample_lens, cum_len, path[:, d])
        return result

    def _online_lookahead_distance(self, task_distance: float) -> float:
        """返回在线路径跟踪的弧长前视距离。"""
        cfg = self.config
        base = max(
            float(getattr(cfg, "leader_max_vel", 1.0)) * 0.6,
            float(getattr(cfg, "wp_radius", 0.5)) * 1.2,
            float(self.grid.resolution) * 2.0,
        )
        horizon_cap = max(float(self.grid.resolution) * 2.0, float(cfg.planner_horizon) * 0.35)
        return max(float(self.grid.resolution), min(base, horizon_cap, max(task_distance, self.grid.resolution)))

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

    def _select_online_target(
        self,
        path: np.ndarray,
        position: np.ndarray,
        task_goal: np.ndarray,
        previous_idx: int,
    ) -> tuple[np.ndarray, int]:
        """沿局部路径按弧长前视选目标，避免逐密集路径点爬行。"""
        path = np.asarray(path, dtype=float)
        position = np.asarray(position, dtype=float)
        task_goal = np.asarray(task_goal, dtype=float)
        if len(path) == 0:
            return task_goal.copy(), 0
        if len(path) == 1:
            return path[0].copy(), 0

        task_distance = float(np.linalg.norm(task_goal - position))
        tracking_clearance = max(float(self._collision_margin) + 0.03, float(self.config.safety_margin))
        if task_distance <= max(self.config.wp_radius, self.grid.resolution):
            return task_goal.copy(), len(path) - 1
        if self._segment_is_safe(np.vstack([position, task_goal]), tracking_clearance):
            return task_goal.copy(), len(path) - 1

        search_start = max(0, min(int(previous_idx), len(path) - 1) - 6)
        rel = path[search_start:]
        closest_i = search_start + int(np.argmin(np.linalg.norm(rel - position, axis=1)))

        lookahead = self._online_lookahead_distance(task_distance)
        remaining = lookahead
        target = path[closest_i].copy()
        target_i = closest_i

        for idx in range(closest_i, len(path) - 1):
            left = path[idx]
            right = path[idx + 1]
            seg_len = float(np.linalg.norm(right - left))
            if seg_len < 1e-9:
                target_i = idx + 1
                target = right.copy()
                continue
            if remaining <= seg_len:
                ratio = remaining / seg_len
                target = left + (right - left) * ratio
                target_i = idx + 1
                break
            remaining -= seg_len
            target = right.copy()
            target_i = idx + 1

        if self._segment_is_safe(np.vstack([position, target]), tracking_clearance):
            return target.copy(), target_i

        # 拐角处禁止直线抄近路：退回到从当前位置可见的最远路径点。
        for idx in range(min(target_i, len(path) - 1), closest_i, -1):
            candidate = path[idx]
            if self._segment_is_safe(np.vstack([position, candidate]), tracking_clearance):
                return candidate.copy(), idx

        next_i = min(closest_i + 1, len(path) - 1)
        return path[next_i].copy(), next_i

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
        obstacles = self.obstacles
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
                        last_sensor_reading = self.sensor.sense(leader_pos, obstacles)
                        last_sensor_time = time_now
                        self.sensor_logs.append(last_sensor_reading.copy())
                    sensor_reading = last_sensor_reading

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
                    obstacles=obstacles,
                    desired_offsets=offsets,
                )
            # 鏀硅繘 APF锛氶殰纰嶇墿鏂ュ姏锛堝惈鐩爣璺濈琛板噺 + 灞€閮ㄦ瀬灏忓€奸€冮€革級
            leader_sdf = float(obstacles.signed_distance(leader_pos))
            if leader_sdf < self.apf.r_rep:
                leader_repulsion_acc = self.apf.compute_avoidance_acceleration(
                    leader_pos, target_wp, obstacles)
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

            # 棰嗚埅鏈虹鎾炴娴?
            if step_idx % collision_check_steps == 0 and obstacles.is_collision(leader_pos_new, inflate=collision_margin):
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
                if float(obstacles.signed_distance(follower_current_pos)) < self.apf.r_rep or other_positions:
                    repulsion_acc = self.apf.compute_avoidance_acceleration(
                        follower_current_pos, target_pos, obstacles,
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

                # 浠庢満纰版挒妫€娴?
                if step_idx % collision_check_steps == 0 and obstacles.is_collision(follower_pos, inflate=collision_margin):
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
            "executed_path": executed_arr,
            "replan_events": self.replan_events,
            "planning_events": self.planning_events,
            "waypoint_events": waypoint_events,
            "formation_adaptation_events": self.formation_adaptation_events,
            "sensor_logs": np.array(self.sensor_logs, dtype=float) if self.sensor_logs else None,
            "collision_log": self.collision_log,
            "fault_log": self.fault_log,
            "safety_metrics": safety_metrics,
        }
