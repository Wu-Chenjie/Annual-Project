"""障碍物场景测试套件。

用途
----
验证障碍物建模、路径规划、避障仿真各模块功能正确性，
并检查厘米级编队误差与零碰撞指标。

原理
----
1) 单元测试：障碍物 SDF、占据栅格、规划算法正确性。
2) 集成测试：完整障碍物仿真，指标阈值法判定通过/失败。

"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np

# 确保 next_project 在路径中。
_here = Path(__file__).resolve().parent
_project = _here.parent
if str(_project) not in sys.path:
    sys.path.insert(0, str(_project))

from core.obstacles import AABB, Sphere, Cylinder, ObstacleField, OccupancyGrid
from core.sensors import RangeSensor6
from core.topology import FormationTopology
from core.planning import AStar, Dijkstra, RRTStar, HybridAStar, PlannerError
from core.planning.firi import FIRIRefiner, FIRICorridor
from core.map_loader import load_from_json
from simulations.formation_simulation import SimulationConfig
from simulations.obstacle_scenario import ObstacleScenarioSimulation
from config import get_config


# ─── 单元测试 ───────────────────────────────────────────


def test_aabb_sdf():
    """AABB 符号距离正确性。"""
    aabb = AABB(np.array([0, 0, 0]), np.array([2, 2, 2]))
    # 外部点
    assert aabb.signed_distance([3, 1, 1]) == 1.0
    # 内部点
    assert aabb.signed_distance([1, 1, 1]) == -1.0
    # 表面点
    assert abs(aabb.signed_distance([0, 1, 1])) < 1e-9


def test_sphere_sdf():
    """球体符号距离正确性。"""
    s = Sphere(np.array([0, 0, 0]), 1.0)
    assert abs(s.signed_distance([2, 0, 0]) - 1.0) < 1e-9
    assert abs(s.signed_distance([0, 0, 0]) + 1.0) < 1e-9


def test_cylinder_sdf():
    """圆柱符号距离正确性。"""
    c = Cylinder(np.array([0, 0]), 0.5, (0, 2))
    assert abs(c.signed_distance([1, 0, 1]) - 0.5) < 1e-9
    assert c.signed_distance([0, 0, 1]) < 0  # 内部


def test_obstacle_field():
    """ObstacleField 碰撞检测。"""
    f = ObstacleField()
    f.add_aabb([0, 0, 0], [2, 2, 2])
    f.add_sphere([5, 0, 1], 0.5)
    assert f.is_collision(np.array([1, 1, 1]))  # 在AABB内
    assert not f.is_collision(np.array([3, 3, 3]))  # 外部
    assert f.is_collision(np.array([5.2, 0, 1]), inflate=0.1)  # 膨胀后碰撞


def test_occupancy_grid():
    """占据栅格索引转换正确性。"""
    grid = OccupancyGrid(
        origin=np.array([0, 0, 0]),
        resolution=0.5,
        shape=(20, 20, 10),
    )
    idx = grid.world_to_index([2.3, 4.7, 1.2])
    assert idx == (4, 9, 2) or idx == (5, 9, 2)  # 取整方向
    world = grid.index_to_world((4, 9, 2))
    assert abs(world[0] - 2.0) < 0.5


def test_sensor_range():
    """六向传感器基本测距。"""
    f = ObstacleField()
    f.add_aabb([3, -0.5, -1], [4, 0.5, 5])
    sensor = RangeSensor6(max_range=10.0, noise_std=0.0, seed=0)
    reading = sensor.sense(np.array([0, 0, 2]), f)
    # +x 方向应检测到 3m 处的障碍物
    assert 2.8 < reading[0] < 3.2
    # -x 方向无障碍 → max_range
    assert reading[1] > 9.0


def test_sensor_range_matches_surface_contact_for_sphere_and_cylinder():
    """解析求交应返回到障碍物表面的距离，而不是步进近似值。"""
    field = ObstacleField()
    field.add_sphere([4.0, 0.0, 2.0], 1.0)
    field.add_cylinder([0.0, 4.0], 0.5, 1.0, 3.0)
    sensor = RangeSensor6(max_range=10.0, noise_std=0.0, seed=0)
    reading = sensor.sense(np.array([0.0, 0.0, 2.0], dtype=float), field)
    assert abs(reading[0] - 3.0) < 1e-6
    assert abs(reading[2] - 3.5) < 1e-6


def test_envelope_radius():
    """编队包络标量接口应与分轴包络保持一致。"""
    topo = FormationTopology(num_followers=3, spacing=1.0)
    r_v = topo.envelope_radius("v_shape")
    r_d = topo.envelope_radius("diamond")
    lateral_v, _, _ = topo.envelope_per_axis("v_shape")
    lateral_d, _, _ = topo.envelope_per_axis("diamond")
    assert lateral_d < lateral_v
    assert r_v == max(topo.envelope_per_axis("v_shape"))
    assert r_d == max(topo.envelope_per_axis("diamond"))
    assert r_d > 1.0


def test_envelope_per_axis_preserves_line_lateral_clearance():
    """线形队形应保持较小横向包络，不能被纵向长度放大。"""
    topo = FormationTopology(num_followers=3, spacing=6.0, arm_length=0.2)
    lateral, longitudinal, vertical = topo.envelope_per_axis("line")
    assert abs(lateral - 0.2) < 1e-9
    assert abs(vertical - 0.2) < 1e-9
    assert abs(longitudinal - 18.2) < 1e-9
    assert topo.envelope_radius("line") == longitudinal


def test_astar_basic():
    """A* 在无障碍地图上找到直线路径。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    grid = f.to_voxel_grid(bounds, 0.5)
    astar = AStar()
    path = astar.plan(np.array([1, 5, 1]), np.array([9, 5, 1]), grid)
    assert len(path) >= 2
    # 无障碍时路径应接近直线
    dist = np.linalg.norm(path[-1] - path[0])
    assert dist < 12


def test_astar_avoidance():
    """A* 能避开中间障碍物。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    f.add_cylinder([5, 5], 1.0, 0, 4)
    grid = f.to_voxel_grid(bounds, 0.5).inflate(0.3)
    astar = AStar()
    path = astar.plan(np.array([1, 5, 1]), np.array([9, 5, 1]), grid)
    # 路径不应穿过障碍中心
    for wp in path:
        assert not f.is_collision(wp, inflate=0.3)


def test_dijkstra_vs_astar():
    """Dijkstra 与 A* 在无障碍地图上路径接近。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    grid = f.to_voxel_grid(bounds, 0.5)
    p_astar = AStar().plan([1, 5, 1], [9, 5, 1], grid)
    p_dijkstra = Dijkstra().plan([1, 5, 1], [9, 5, 1], grid)
    assert len(p_astar) == len(p_dijkstra)


def test_rrt_star_feasible():
    """RRT* 在简单场景中找到可行路径。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    f.add_cylinder([5, 5], 1.0, 0, 4)
    grid = f.to_voxel_grid(bounds, 0.5).inflate(0.3)
    rrt = RRTStar(max_iter=3000, rewire_radius=2.0, goal_sample_rate=0.2)
    path = rrt.plan(np.array([1, 5, 1]), np.array([9, 5, 1]), grid, seed=123)
    assert len(path) >= 2, f"RRT* 路径点数={len(path)}"
    for wp in path:
        assert not f.is_collision(wp, inflate=0.3)


def test_map_loader():
    """地图 JSON 加载成功。"""
    map_path = _project / "maps" / "sample_simple.json"
    if map_path.exists():
        field, bounds = load_from_json(str(map_path))
        assert len(field) > 0
        assert bounds.shape == (2, 3)


def test_smooth_path():
    """路径平滑增加航点数。"""
    raw = np.array([[0, 0, 0], [5, 0, 0], [10, 0, 0]], dtype=float)
    smoothed = AStar().smooth(raw, num_insert=4)
    assert len(smoothed) > len(raw)


def test_firi_refine_preserves_clearance():
    """FIRI 精修应保持路径点不进入膨胀障碍区。"""
    f = ObstacleField()
    f.add_cylinder([5, 0], 0.8, 0, 4)
    refiner = FIRIRefiner(f, min_clearance=0.4)
    raw = np.array([
        [1.0, 0.0, 1.0],
        [4.8, 0.0, 1.0],
        [9.0, 0.0, 1.0],
    ], dtype=float)
    refined = refiner.refine(raw, seeds=raw)
    assert len(refined) >= len(raw)
    for point in refined:
        assert f.signed_distance(point) >= 0.4 - 1e-6


def test_firi_corridor_projection_satisfies_halfspaces():
    """半空间走廊投影后应满足 A x <= b。"""
    A = np.array([
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=float)
    b = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
    corridor = FIRICorridor(
        A=A,
        b=b,
        start=np.array([0.0, 0.0, 0.0], dtype=float),
        goal=np.array([0.5, 0.5, 0.0], dtype=float),
        min_clearance=0.2,
    )
    projected = corridor.project(np.array([3.0, 2.0, 0.0], dtype=float))
    assert np.all(A @ projected <= b + 1e-8)


def test_firi_analytic_plane_and_mvie_fallback():
    """解析分离平面应生成走廊，并带有 MVIE/内切球信息。"""
    f = ObstacleField()
    f.add_sphere([5.0, 1.5, 1.0], 0.5)
    refiner = FIRIRefiner(f, min_clearance=0.3, mvie_enabled=True)
    start = np.array([1.0, 0.0, 1.0], dtype=float)
    goal = np.array([9.0, 0.0, 1.0], dtype=float)
    corridor = refiner._build_segment_corridor(start, goal)
    assert corridor.A.shape[0] > 6
    assert corridor.contains(start)
    assert corridor.contains(goal)
    assert corridor.ellipsoid_center is not None
    assert corridor.ellipsoid_shape is not None
    assert corridor.ellipsoid_shape.shape == (3, 3)


# ─── 集成测试 ───────────────────────────────────────────


def test_obstacle_simulation_zero_collision():
    """集成：离线 A* 规划 + 简单地图，零碰撞 + 厘米级误差。"""
    config = SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        follower_gain_scale=1.0,
        wp_radius=0.06,
        wp_radius_final=0.03,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()

    collisions = len(result.get("collision_log", []))
    means = result["metrics"]["mean"]
    maxs = result["metrics"]["max"]

    assert collisions == 0, f"碰撞次数={collisions}，期望 0"
    for i in range(len(means)):
        assert means[i] < 0.015, f"F{i+1} 均值={means[i]:.4f}m > 1.5cm"
        assert maxs[i] < 0.06, f"F{i+1} 最大={maxs[i]:.4f}m > 6cm"


def test_terminal_hold_reduces_end_jitter():
    """终端保持应压制终点附近快速抖动。"""
    config = SimulationConfig(
        max_sim_time=18.0,
        use_smc=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        follower_gain_scale=1.0,
        wp_radius=0.06,
        wp_radius_final=0.03,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    leader = np.asarray(result["leader"], dtype=float)
    assert len(leader) > 20
    tail = leader[-20:]
    tail_span = float(np.max(np.linalg.norm(tail - tail.mean(axis=0, keepdims=True), axis=1)))
    assert tail_span < 0.5, f"终点段抖动过大: {tail_span:.3f}m"
    drift = float(np.linalg.norm(tail[-1] - tail[0]))
    assert drift < 0.25, f"终点段仍有明显漂移: {drift:.3f}m"


def test_online_mode_preserves_task_waypoint_semantics():
    """在线模式不应把任务航点语义替换成局部重规划点。"""
    config = SimulationConfig(
        max_sim_time=8.0,
        use_smc=True,
        use_backstepping=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        follower_gain_scale=1.0,
        wp_radius=0.4,
        wp_radius_final=0.2,
        leader_acc_alpha=0.35,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.3,
        planner_replan_interval=0.4,
        planner_horizon=4.0,
        safety_margin=0.3,
        sensor_enabled=True,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([10.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    assert len(result["task_waypoints"]) == 3
    assert result["completed_waypoint_count"] <= 3
    assert len(result["replanned_waypoints"]) >= 2


def test_named_offline_presets_are_offline():
    """不带 _online 的命名场景应保持离线模式，避免误用在线终止逻辑。"""
    assert get_config("school_corridor").planner_mode == "offline"
    assert get_config("meeting_room").planner_mode == "offline"
    assert get_config("laboratory").planner_mode == "offline"


def test_online_setup_keeps_task_waypoints_as_execution_waypoints():
    """在线模式初始化不应再用离线参考路径覆盖任务航点序列。"""
    cfg = get_config("school_corridor_online")
    cfg.max_sim_time = 1.0
    cfg.danger_mode_enabled = False
    sim = ObstacleScenarioSimulation(cfg)
    assert len(sim.waypoints) == len(sim._planning_waypoints)
    assert sim.planned_path is not None
    assert len(sim.planned_path) > len(sim._planning_waypoints)
    np.testing.assert_allclose(np.asarray(sim.waypoints), np.asarray(sim._planning_waypoints))


def test_meeting_room_offline_reference_is_collision_free():
    """会议室离线参考路径不应再在段规划失败时用直线穿障碍兜底。"""
    cfg = get_config("meeting_room")
    sim = ObstacleScenarioSimulation(cfg)
    assert sim.planned_path is not None
    assert sim._path_segment_clearance(sim.planned_path, 0.0) >= 0.0


def test_company_online_reports_geometrically_blocked_waypoints():
    """公司格子间在线场景的被覆盖任务点应被投影，而不是误报为可达。"""
    cfg = get_config("company_cubicles_online")
    cfg.max_sim_time = 1.0
    sim = ObstacleScenarioSimulation(cfg)
    raw = np.asarray(sim._task_waypoints, dtype=float)
    safe = np.asarray(sim._planning_waypoints, dtype=float)
    blocked = [i for i, point in enumerate(raw) if sim.obstacles.signed_distance(point) < 0.0]
    assert blocked, "该场景应包含几何不可达任务航点"
    assert any(float(np.linalg.norm(raw[i] - safe[i])) > cfg.wp_radius for i in blocked)


def test_online_window_path_extends_to_task_goal():
    """在线窗口重规划结果应拼接回任务航点，避免到窗口末端后停止。"""
    cfg = get_config("meeting_room_online")
    cfg.max_sim_time = 1.0
    sim = ObstacleScenarioSimulation(cfg)
    window_path = np.array([
        sim._planning_waypoints[0],
        np.array([1.0, 2.4, 1.8], dtype=float),
    ])
    task_goal = sim._planning_waypoints[1]
    extended = sim._extend_online_path_to_task(window_path, task_goal)
    assert len(extended) > len(window_path)
    assert np.linalg.norm(extended[-1] - task_goal) <= max(cfg.wp_radius, cfg.planner_resolution)
    assert sim._segment_is_safe(extended, sim._collision_margin)


def test_online_accept_path_rejects_unsafe_clearance_without_fallback():
    cfg = SimulationConfig(
        max_sim_time=1.0,
        use_smc=True,
        num_followers=1,
        enable_obstacles=True,
        planner_kind="astar",
        planner_mode="online",
        planner_resolution=0.25,
        safety_margin=0.3,
        plan_clearance_extra=0.2,
        sensor_enabled=False,
        obstacle_field=ObstacleField(),
        waypoints=[
            np.array([0.0, 0.0, 1.0], dtype=float),
            np.array([4.0, 0.0, 1.0], dtype=float),
        ],
    )
    cfg.obstacle_field.add_sphere([2.0, 0.0, 1.0], 0.35)
    sim = ObstacleScenarioSimulation(cfg)

    class NoopRefiner:
        def refine(self, path, seeds=None):
            return np.asarray(path, dtype=float)

    sim.firi_refiner = NoopRefiner()
    sim._plan_segment_fallback = lambda *args, **kwargs: None
    unsafe_path = np.array([
        [0.0, 0.0, 1.0],
        [4.0, 0.0, 1.0],
    ], dtype=float)

    accepted = sim._accept_online_path(
        unsafe_path,
        leader_pos=np.array([0.0, 0.0, 1.0], dtype=float),
        task_goal=np.array([4.0, 0.0, 1.0], dtype=float),
        time_now=1.25,
    )

    assert accepted is None
    assert sim.replan_events[-1]["mode"] == "clearance_blocked"
    assert sim.replan_events[-1]["reason"] == "no_continuous_clearance_path"


def test_inflate_radius_uses_runtime_formation_envelope():
    """规划膨胀半径应跟随运行时队形，而不是固定 initial_formation。"""
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=3,
        formation_spacing=6.0,
        initial_formation="diamond",
        enable_obstacles=True,
        planner_use_formation_envelope=True,
        planner_resolution=0.5,
        safety_margin=0.5,
        obstacle_field=ObstacleField(),
        waypoints=[
            np.array([0.0, 0.0, 1.0], dtype=float),
            np.array([4.0, 0.0, 1.0], dtype=float),
        ],
    )
    sim = ObstacleScenarioSimulation(cfg)
    baseline = sim._inflate_r()
    sim.topology.switch_formation("line", transition_time=1.0)
    narrowed = sim._inflate_r()
    assert narrowed < baseline
    assert abs(narrowed - (sim.topology.envelope_per_axis()[0] + cfg.safety_margin)) < 1e-9


def test_online_auto_shrink_rebuilds_grid_and_switches_to_line():
    """窄通道测距应触发自动收缩，并刷新规划网格膨胀层。"""
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=3,
        formation_spacing=6.0,
        initial_formation="diamond",
        enable_obstacles=True,
        planner_mode="online",
        planner_use_formation_envelope=True,
        planner_resolution=0.5,
        safety_margin=0.5,
        sensor_enabled=False,
        obstacle_field=ObstacleField(),
        waypoints=[
            np.array([0.0, 0.0, 1.0], dtype=float),
            np.array([4.0, 0.0, 1.0], dtype=float),
        ],
    )
    sim = ObstacleScenarioSimulation(cfg)
    before = sim.grid.data.copy()
    changed = sim.topology.auto_shrink((0.3, 40.0, 5.0))
    assert changed
    sim._rebuild_planning_grid()
    after = sim.grid.data
    assert sim.topology.current_formation == "line" or sim.topology._target_formation == "line"
    assert int(np.count_nonzero(after >= 1)) <= int(np.count_nonzero(before >= 1))


def test_formation_apf_switch_is_no_longer_dead_config():
    """apf_formation_centroid 打开后应真正构建编队势场组件。"""
    cfg = SimulationConfig(
        max_sim_time=1.0,
        num_followers=2,
        formation_spacing=0.5,
        initial_formation="diamond",
        enable_obstacles=True,
        obstacle_field=ObstacleField(),
        planner_kind="astar",
        planner_mode="offline",
        safety_margin=0.3,
        apf_formation_centroid=True,
        apf_centroid_alpha=0.25,
        apf_centroid_beta=0.75,
        waypoints=[
            np.array([0.0, 0.0, 1.0], dtype=float),
            np.array([4.0, 0.0, 1.0], dtype=float),
        ],
    )
    cfg.obstacle_field.add_sphere([1.0, 0.0, 1.0], 0.2)
    sim = ObstacleScenarioSimulation(cfg)
    assert sim.formation_apf is not None
    assert abs(sim.formation_apf.alpha - 0.25) < 1e-9
    assert abs(sim.formation_apf.beta - 0.75) < 1e-9

    leader_pos = np.array([0.0, 0.0, 1.0], dtype=float)
    follower_positions = [np.array([-0.5, -0.5, 1.0], dtype=float), np.array([-0.5, 0.5, 1.0], dtype=float)]
    desired_offsets = sim.topology.get_offsets("diamond")
    leader_force, follower_forces = sim.formation_apf.compute_formation_avoidance(
        leader_pos=leader_pos,
        follower_positions=follower_positions,
        goal=np.array([4.0, 0.0, 1.0], dtype=float),
        obstacles=cfg.obstacle_field,
        desired_offsets=desired_offsets,
    )
    assert leader_force.shape == (3,)
    assert len(follower_forces) == 2
    assert any(float(np.linalg.norm(force)) > 0.0 for force in follower_forces)


def test_cpp_formation_apf_switch_changes_probe_behavior():
    """C++ 路径中 apf_formation_centroid 打开后应改变探针场景结果。"""
    root = Path(__file__).resolve().parent.parent
    build_dir = root / "cpp" / "build"
    exe = build_dir / "sim_apf_formation_probe.exe"
    if os.name != "nt":
        exe = build_dir / "sim_apf_formation_probe"

    subprocess.run(
        ["cmake", "--build", str(build_dir), "--target", "sim_apf_formation_probe"],
        cwd=root / "cpp",
        check=True,
        capture_output=True,
        text=True,
    )

    off = subprocess.run(
        [str(exe), "off"],
        cwd=root / "cpp",
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    on = subprocess.run(
        [str(exe), "on"],
        cwd=root / "cpp",
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()

    def parse(line: str) -> dict[str, float | str]:
        parts: dict[str, float | str] = {}
        for token in line.split():
            key, value = token.split("=", 1)
            if key == "centroid":
                parts[key] = value
            else:
                parts[key] = float(value)
        return parts

    off_metrics = parse(off)
    on_metrics = parse(on)
    assert off_metrics["centroid"] == "off"
    assert on_metrics["centroid"] == "on"
    delta = (
        abs(float(off_metrics["path_len"]) - float(on_metrics["path_len"]))
        + abs(float(off_metrics["final_y"]) - float(on_metrics["final_y"]))
        + abs(float(off_metrics["follower0_final_err"]) - float(on_metrics["follower0_final_err"]))
    )
    assert delta > 1e-6, (off, on)


def test_safe_follower_target_shrinks_blocked_offset():
    """从机目标若被障碍物覆盖，应沿 leader 方向收缩到安全位置。"""
    cfg = get_config("company_cubicles_online")
    cfg.max_sim_time = 1.0
    sim = ObstacleScenarioSimulation(cfg)
    leader_pos = np.array([4.0, 3.8, 2.5], dtype=float)
    raw_target = np.array([4.5, 3.8, 2.5], dtype=float)
    target = sim._safe_follower_target(leader_pos, raw_target)
    assert sim.obstacles.signed_distance(target) >= sim._collision_margin
    assert sim.obstacles.signed_distance(raw_target) < sim._collision_margin
    assert np.linalg.norm(target - raw_target) > 1e-6


def test_obstacle_simulation_dijkstra():
    """集成：Dijkstra 离线规划 + 零碰撞。"""
    config = SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        wp_radius=0.06,
        wp_radius_final=0.03,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="dijkstra",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    assert len(result.get("collision_log", [])) == 0


def test_rrt_star_obstacle():
    """集成：RRT* 离线规划 + 零碰撞。"""
    config = SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        wp_radius=0.06,
        wp_radius_final=0.03,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="rrt_star",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    collisions = len(result.get("collision_log", []))
    means = result["metrics"]["mean"]
    assert collisions == 0, f"RRT* 碰撞={collisions}"
    assert all(m < 0.06 for m in means), f"RRT* 均值误差过大: {means}"


def test_hybrid_astar_basic():
    """HybridA* 在无障碍地图上找到可行路径。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    grid = f.to_voxel_grid(bounds, 0.5)
    ha = HybridAStar(v_max=2.0, max_iter=5000)
    path = ha.plan(np.array([1, 5, 1]), np.array([9, 5, 1]), grid, seed=42)
    assert len(path) >= 2
    dist = np.linalg.norm(path[-1] - path[0])
    assert 7.0 < dist < 12.0  # 路径长度在合理范围


def test_hybrid_astar_avoidance():
    """HybridA* 能避开中间障碍物。"""
    bounds = np.array([[0, 0, 0], [10, 10, 4]])
    f = ObstacleField()
    f.add_cylinder([5, 5], 1.0, 0, 4)
    grid = f.to_voxel_grid(bounds, 0.5).inflate(0.3)
    ha = HybridAStar(v_max=2.0, max_iter=8000)
    path = ha.plan(np.array([1, 5, 1]), np.array([9, 5, 1]), grid, seed=123)
    assert len(path) >= 3, f"HybridA* 路径点数={len(path)}"
    for wp in path:
        assert not f.is_collision(wp, inflate=0.3), f"路径点 {wp} 碰撞"


def test_hybrid_astar_obstacle():
    """集成：HybridA* 离线规划 + 零碰撞。"""
    config = SimulationConfig(
        max_sim_time=40.0,
        use_smc=True,
        num_followers=2,
        formation_spacing=0.3,
        initial_formation="diamond",
        leader_max_vel=0.4,
        leader_max_acc=0.5,
        leader_gain_scale=0.8,
        wp_radius=0.06,
        wp_radius_final=0.03,
        enable_obstacles=True,
        map_file=str(_project / "maps" / "sample_simple.json"),
        planner_kind="hybrid_astar",
        planner_mode="offline",
        planner_resolution=0.3,
        safety_margin=0.3,
        waypoints=[
            np.array([2.0, 10.0, 2.0], dtype=float),
            np.array([20.0, 10.0, 2.0], dtype=float),
        ],
    )

    sim = ObstacleScenarioSimulation(config=config)
    result = sim.run()
    collisions = len(result.get("collision_log", []))
    means = result["metrics"]["mean"]
    assert collisions == 0, f"HybridA* 碰撞={collisions}"
    assert all(m < 0.06 for m in means), f"HybridA* 均值误差过大: {means}"


def test_regression_baseline():
    """回归：无障碍仿真误差与基线一致。"""
    config = SimulationConfig(
        max_sim_time=20.0,
        use_smc=True,
        num_followers=3,
    )
    from simulations.formation_simulation import FormationSimulation
    sim = FormationSimulation(config=config)
    result = sim.run()
    maxs = result["metrics"]["max"]
    assert all(m < 0.3 for m in maxs), f"基线退化：max={maxs}"


# ─── 运行入口 ───────────────────────────────────────────


if __name__ == "__main__":
    tests = [
        ("AABB SDF", test_aabb_sdf),
        ("Sphere SDF", test_sphere_sdf),
        ("Cylinder SDF", test_cylinder_sdf),
        ("ObstacleField", test_obstacle_field),
        ("OccupancyGrid", test_occupancy_grid),
        ("Sensor Range", test_sensor_range),
        ("Envelope Radius", test_envelope_radius),
        ("A* Basic", test_astar_basic),
        ("A* Avoidance", test_astar_avoidance),
        ("Dijkstra vs A*", test_dijkstra_vs_astar),
        ("RRT* Feasible", test_rrt_star_feasible),
        ("Map Loader", test_map_loader),
        ("Path Smooth", test_smooth_path),
        ("Integration: A* Zero Collision", test_obstacle_simulation_zero_collision),
        ("Integration: Dijkstra", test_obstacle_simulation_dijkstra),
        ("Integration: RRT*", test_rrt_star_obstacle),
        ("Hybrid A* Basic", test_hybrid_astar_basic),
        ("Hybrid A* Avoidance", test_hybrid_astar_avoidance),
        ("Integration: Hybrid A*", test_hybrid_astar_obstacle),
        ("Regression: Baseline", test_regression_baseline),
    ]

    passed = 0
    failed = 0
    for name, func in tests:
        try:
            func()
            print(f"  PASS  {name}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {name}: {e}")
            failed += 1

    print(f"\n{passed}/{len(tests)} 通过, {failed} 失败")
    if failed > 0:
        sys.exit(1)
