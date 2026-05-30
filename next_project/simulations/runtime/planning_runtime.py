from __future__ import annotations

import time

import numpy as np

from core.artificial_potential_field import ImprovedArtificialPotentialField
from core.formation_clearance import FormationClearancePolicy
from core.map_loader import load_from_json, load_from_npz
from core.obstacles import ObstacleField, SDFAwareGrid
from core.planning import (
    AStar,
    CostAwareGrid,
    Dijkstra,
    DualModeScheduler,
    FormationAPF,
    GNNPlanner,
    HybridAStar,
    InformedRRTStar,
    Planner,
    RRTStar,
    RiskAdaptiveReplanInterval,
    TrajectoryOptimizer,
    TurnConstrainedAStar,
    WindowReplanner,
)
from core.planning.firi import FIRIRefiner
from core.planning.mpc_tracker import MPCFeasibilityEvaluator
from core.sensors import RangeSensor6


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


class PlanningRuntime:
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
        # 未知地图场景中 APF 同样工作：排斥力来自传感器已发现的动态障碍场
        return ImprovedArtificialPotentialField(**params)


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
        self._planner_initial_map_unknown = bool(getattr(cfg, "planner_initial_map_unknown", False))
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

        self._planner_obstacles = ObstacleField() if self._planner_initial_map_unknown else self.obstacles
        self._discovered_obstacles = ObstacleField()  # grows as sensors discover cells
        if self._planner_initial_map_unknown and hasattr(self, "grid"):
            self.grid = None

        # 浣撶礌鍖栵紙濡傚皻鏈粠 NPZ 鍔犺浇锛?
        if not hasattr(self, "grid") or self.grid is None:
            self.grid = self._planner_obstacles.to_voxel_grid(self._map_bounds, cfg.planner_resolution)
            self.grid = self.grid.inflate(self._inflate_margin_xyz())

        # SDF 鎰熺煡鍖呰锛氱簿鍒よ杽闅滅鐗╋紙鍗婂緞 < 鏍呮牸鍒嗚鲸鐜囷級锛岄伩鍏嶄綋绱犲寲涓㈠け
        if getattr(cfg, "planner_sdf_aware", False) and not self._planner_initial_map_unknown:
            clearance = max(float(cfg.safety_margin), float(getattr(self, "_collision_margin", 0.0)))
            self.grid = SDFAwareGrid(
                self.grid,
                self._planner_obstacles,
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


    def _planning_signed_distance(self, point: np.ndarray) -> float:
        """Signed distance visible to the planner, not necessarily the full truth map."""
        self._sdf_query_count += 1
        if getattr(self, "_planner_initial_map_unknown", False):
            idx = self.grid.world_to_index(np.asarray(point, dtype=float))
            return -float(self.grid.resolution) if self.grid.is_occupied(idx) else float("inf")
        return float(self.obstacles.signed_distance(np.asarray(point, dtype=float)))


    @staticmethod
    def _planning_event_payload(event: dict) -> dict:
        payload = dict(event)
        if "wall_time_ms" not in payload and "wall_time_s" in payload:
            payload["wall_time_ms"] = float(payload["wall_time_s"]) * 1000.0
        if "path_points" not in payload and "point_count" in payload:
            payload["path_points"] = int(payload["point_count"])
        return payload


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
            signed_distance=lambda p: float(self._planning_signed_distance(np.asarray(p, dtype=float))),
            follower_offsets=offsets,
            base_clearance=self._formation_clearance_base(clearance),
            sample_spacing=sample_spacing,
            formation_aware=self._formation_clearance_enabled() if formation_aware is None else bool(formation_aware),
        )


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
        base_grid = self._planner_obstacles.to_voxel_grid(self._map_bounds, self.config.planner_resolution)
        plan_grid = base_grid.inflate(self._inflate_margin_xyz())
        if getattr(self.config, "planner_sdf_aware", False) and not getattr(self, "_planner_initial_map_unknown", False):
            clearance = max(float(self.config.safety_margin), float(getattr(self, "_collision_margin", 0.0)))
            plan_grid = SDFAwareGrid(
                plan_grid,
                self._planner_obstacles,
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
            self.replanner._sensor_grid_dirty = False
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
            self._planner_obstacles,
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
        self.mpc_feasibility = None
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
                    danger_planner._lazy_obstacles = (
                        self._discovered_obstacles if self._planner_initial_map_unknown else self.obstacles
                    )
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
                obstacle_field=self._discovered_obstacles if getattr(cfg, "planner_initial_map_unknown", False) else self.obstacles,
            )
            self.replanner.path_refiner = self.firi_refiner

            if cfg.sensor_enabled:
                self.sensor = RangeSensor6(
                    max_range=cfg.planner_horizon,
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

        point_sd = float(self._planning_signed_distance(point))
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
                        sd = float(self._planning_signed_distance(candidate))
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
        self._clearance_check_count += 1
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
            return float(self._planning_signed_distance(path[0]))
        sample_spacing = float(spacing or max(min(self.grid.resolution * 0.5, 0.10), 0.08))
        worst = float("inf")
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            dist = float(np.linalg.norm(b - a))
            n = max(1, int(np.ceil(dist / sample_spacing)))
            for j in range(n + 1):
                point = a + (b - a) * (j / n)
                worst = min(worst, float(self._planning_signed_distance(point)))
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
                min_sd = self._planning_signed_distance(wp)
                worst_pos = wp
                for off in offsets:
                    check_pos = wp + off
                    sd = self._planning_signed_distance(check_pos)
                    if sd < min_sd:
                        min_sd = sd
                        worst_pos = check_pos
                if min_sd >= min_clearance:
                    break
                grad = np.array([
                    self._planning_signed_distance(worst_pos + [eps, 0, 0])
                    - self._planning_signed_distance(worst_pos - [eps, 0, 0]),
                    self._planning_signed_distance(worst_pos + [0, eps, 0])
                    - self._planning_signed_distance(worst_pos - [0, eps, 0]),
                    self._planning_signed_distance(worst_pos + [0, 0, eps])
                    - self._planning_signed_distance(worst_pos - [0, 0, eps]),
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
                min_sd_sp = self._planning_signed_distance(sp)
                for off in offsets:
                    sd = self._planning_signed_distance(sp + off)
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
            corridors = None
            if getattr(self.config, "firi_enabled", False) and hasattr(self, "firi_refiner"):
                corridors = self.firi_refiner.build_corridors_for_path(planned)
            self.planned_trajectory = self.trajectory_optimizer.optimize(
                planned,
                method=getattr(self.config, "trajectory_optimizer_method", "moving_average"),
                clearance_checker=lambda candidate: self._segment_is_safe(candidate, min_clearance),
                corridors=corridors,
                fallback_to_raw=True,
            )
            self.mpc_feasibility = self._evaluate_mpc_feasibility()
            return np.asarray(self.planned_trajectory.positions, dtype=float)
        self.planned_trajectory = None
        self.mpc_feasibility = None
        return planned


    def _evaluate_mpc_feasibility(self):
        if self.planned_trajectory is None or not getattr(self.config, "mpc_feasibility_enabled", True):
            return None
        evaluator = MPCFeasibilityEvaluator(
            max_speed=getattr(self.config, "leader_max_vel", 4.0),
            max_acceleration=getattr(self.config, "leader_max_acc", 5.0),
            rms_limit=getattr(self.config, "mpc_feasibility_rms_limit", 0.75),
        )
        return evaluator.evaluate_trajectory(self.planned_trajectory)


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
