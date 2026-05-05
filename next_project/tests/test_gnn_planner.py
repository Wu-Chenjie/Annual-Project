"""GNN 规划器与可见图单测（论文2）。

验证:
- 可见图构建与顶点数
- GNN 活动场单调性
- 路径提取有效性
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.planning.visibility_graph import VisibilityGraph, fibonacci_sphere
from core.planning.gnn_planner import GNNPlanner


class MockObstacle:
    def __init__(self, center, half_extents, kind="aabb"):
        c = np.array(center, dtype=float)
        h = np.array(half_extents, dtype=float)
        self.center = c
        self.half_extents = h
        self.min_corner = c - h
        self.max_corner = c + h


class MockObstacleField:
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def __iter__(self):
        return iter(self.obstacles)

    def signed_distance(self, pos):
        # 简单 SDF：到最近障碍物中心的距离减去半尺寸
        p = np.asarray(pos, dtype=float)
        min_dist = float("inf")
        for obs in self.obstacles:
            d = float(np.linalg.norm(p - obs.center))
            # 粗略：减去最大半尺寸作为表面距离
            r = float(np.max(np.abs(obs.half_extents)))
            min_dist = min(min_dist, d - r)
        return min_dist


def test_fibonacci_sphere():
    """Fibonacci 球面分布点数正确。"""
    pts = fibonacci_sphere(12)
    assert pts.shape == (12, 3)
    # 所有点到原点距离应≈1
    norms = np.linalg.norm(pts, axis=1)
    assert np.allclose(norms, 1.0, atol=1e-6), "球面点应在单位球面上"


def test_visibility_graph_build():
    """可见图构建：含起点终点 + 障碍物顶点。"""
    obstacles = [
        MockObstacle([5.0, 5.0, 5.0], [1.0, 1.0, 1.0]),
        MockObstacle([10.0, 10.0, 5.0], [1.5, 1.5, 2.0]),
    ]
    obs_field = MockObstacleField(obstacles)

    vg = VisibilityGraph(angular_res=8, buffer_zone=0.3)
    start = np.array([0.0, 0.0, 2.0], dtype=float)
    goal = np.array([15.0, 15.0, 5.0], dtype=float)

    vg.build(start, goal, obs_field, visible_range=30.0)

    # 起点 + 终点 + 2个AABB各8个顶点 = 18
    assert vg.num_vertices == 2 + 16, f"预期 18 顶点，实际 {vg.num_vertices}"

    # 起点(0)应可见至少一个邻居
    assert len(vg.adjacency[0]) > 0, "起点应有可见邻居"


def test_gnn_activity_monotonic():
    """GNN 收敛后活动场从目标向外单调递减。"""
    obstacles = [
        MockObstacle([5.0, 5.0, 5.0], [1.0, 1.0, 1.0]),
    ]
    obs_field = MockObstacleField(obstacles)

    vg = VisibilityGraph(angular_res=6, buffer_zone=0.3)
    start = np.array([0.0, 0.0, 2.0], dtype=float)
    goal = np.array([10.0, 10.0, 5.0], dtype=float)

    vg.build(start, goal, obs_field, visible_range=30.0)

    gn = GNNPlanner()
    x = gn._integrate_gnn(
        gn._compute_weight_matrix(vg),
        gn._compute_excitation(vg.vertices, start, goal),
    )

    # 目标顶点(1)活动值应最高
    assert x[1] > x[0], f"目标活动 {x[1]:.4f} 应 > 起点活动 {x[0]:.4f}"

    # 活动场应在 [0, B] 范围
    assert np.all(x >= 0) and np.all(x <= gn.B + 1e-6), \
        f"活动场越界: min={x.min():.4f}, max={x.max():.4f}"


def test_gnn_plan_path():
    """GNN 规划器完整路径提取。"""
    obstacles = [
        MockObstacle([5.0, 5.0, 5.0], [1.0, 1.0, 1.0]),
    ]
    obs_field = MockObstacleField(obstacles)

    vg = VisibilityGraph(angular_res=6, buffer_zone=0.3)
    start = np.array([0.0, 0.0, 2.0], dtype=float)
    goal = np.array([10.0, 10.0, 5.0], dtype=float)

    vg.build(start, goal, obs_field, visible_range=30.0)

    gn = GNNPlanner()
    path = gn.plan(start, goal, None, visibility_graph=vg)

    assert len(path) >= 2, f"路径应至少 2 点，实际 {len(path)}"
    # 起点接近给定起点
    assert np.linalg.norm(path[0] - start) < 1.0, "路径起点应接近给定起点"
    # 终点接近给定目标
    assert np.linalg.norm(path[-1] - goal) < 1.0, "路径终点应接近给定目标"


def test_gnn_extraction_uses_visible_edges_only():
    """贪心卡住时，GNN 回补路径仍必须沿可见图边走。"""
    class ManualGraph:
        vertices = [
            np.array([0.0, 0.0, 0.0], dtype=float),
            np.array([3.0, 0.0, 0.0], dtype=float),
            np.array([1.0, 0.0, 0.0], dtype=float),
            np.array([2.0, 0.0, 0.0], dtype=float),
            np.array([1.0, 1.0, 0.0], dtype=float),
        ]
        adjacency = [
            {2, 4},
            {3},
            {0, 3},
            {2, 1},
            {0},
        ]

    graph = ManualGraph()
    planner = GNNPlanner()
    activity = np.array([0.0, 1.0, 0.4, 0.6, 0.9], dtype=float)
    path = planner._extract_path_from_activities(
        activity,
        graph,
        graph.vertices[0],
        graph.vertices[1],
    )

    vertex_lookup = {tuple(v): i for i, v in enumerate(graph.vertices)}
    indices = [vertex_lookup[tuple(point)] for point in path]
    for left, right in zip(indices, indices[1:]):
        assert right in graph.adjacency[left], f"{left}->{right} 不是可见图边"
    assert indices[-1] == 1


def test_risk_adaptive_interval():
    """风险驱动间隔：高风险缩短间隔，低风险延长间隔。"""
    from core.planning.replanner import RiskAdaptiveReplanInterval

    rai = RiskAdaptiveReplanInterval(base_interval=0.4, min_interval=0.1, max_interval=1.0)

    # 初始 → base
    assert rai.interval == 0.4, "初始应为 base interval"

    # 高风险（SDF=0.1）→ 短间隔
    rai.update(min_sdf=0.1, sensor_min=5.0)
    assert rai.interval < 0.4, f"高风险应缩短间隔，实际 {rai.interval:.3f}"

    # 低风险（SDF=10.0）→ 长间隔
    for _ in range(19):
        rai.update(min_sdf=10.0, sensor_min=10.0)
    assert rai.interval > 0.5, f"低风险应延长间隔，实际 {rai.interval:.3f}"


if __name__ == "__main__":
    tests = [
        ("Fibonacci 球面", test_fibonacci_sphere),
        ("可见图构建", test_visibility_graph_build),
        ("GNN 活动单调性", test_gnn_activity_monotonic),
        ("GNN 路径提取", test_gnn_plan_path),
        ("GNN 可见边回补", test_gnn_extraction_uses_visible_edges_only),
        ("风险驱动间隔", test_risk_adaptive_interval),
    ]
    passed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {name}: {e}")
    print(f"\n{passed}/{len(tests)} 通过")
