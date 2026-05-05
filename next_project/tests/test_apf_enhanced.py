"""改进 APF 增强项单测（论文1）。

验证:
- n_decay 自适应（含缓存命中）
- 通信约束力量纲断言
- Rodrigues 旋转力场切向性
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.artificial_potential_field import ImprovedArtificialPotentialField


class MockObstacles:
    """模拟障碍物场：一个球体障碍物。"""
    def __init__(self):
        self.center = np.array([5.0, 5.0, 5.0], dtype=float)
        self.radius = 1.0

    def signed_distance(self, pos):
        return float(np.linalg.norm(pos - self.center) - self.radius)

    @property
    def obstacles(self):
        return [type('Mock', (), {'center': self.center, 'radius': self.radius})()]


def test_adaptive_n_decay_basic():
    """自适应 n_decay 基本功能：密集 vs 稀疏环境。"""
    apf = ImprovedArtificialPotentialField(adaptive_n_decay=True, r_rep=3.0)
    obstacles = MockObstacles()

    # 远离障碍物 → 低密度 → n≈1
    n_far = apf._adaptive_n_decay(np.array([10.0, 10.0, 10.0], dtype=float), obstacles)
    assert 1.0 <= n_far <= 2.5, f"稀疏环境 n 应接近 1，实际 {n_far:.2f}"

    # 靠近障碍物 → 高密度 → n≈4
    n_near = apf._adaptive_n_decay(np.array([5.5, 5.5, 5.5], dtype=float), obstacles)
    assert n_near > n_far, f"密集环境 n={n_near:.2f} 应 > 稀疏 n={n_far:.2f}"


def test_adaptive_n_decay_cache():
    """自适应 n_decay 缓存：同位置复用。"""
    apf = ImprovedArtificialPotentialField(adaptive_n_decay=True, r_rep=3.0)
    obstacles = MockObstacles()
    pos = np.array([5.0, 5.0, 5.0], dtype=float)

    n1 = apf._adaptive_n_decay(pos, obstacles)
    # 第二次调用，位置未变，应命中缓存
    n2 = apf._adaptive_n_decay(pos, obstacles)
    assert n1 == n2, "缓存命中时 n 应一致"


def test_communication_constraint_force_dim():
    """通信约束力量纲断言：返回值在 [0, k_comm*n_others] 范围。"""
    apf = ImprovedArtificialPotentialField(k_comm=0.3, comm_range=10.0)
    pos = np.array([0.0, 0.0, 0.0], dtype=float)
    others = [np.array([8.0, 0.0, 0.0], dtype=float), np.array([0.0, 8.0, 0.0], dtype=float)]

    f = apf._communication_constraint_force(pos, others)
    norm = float(np.linalg.norm(f))
    assert norm <= apf.k_comm * len(others) + 1e-6, \
        f"通信力 {norm:.4f} 超过上限 {apf.k_comm * len(others)}"


def test_communication_constraint_smooth():
    """通信约束力平滑过渡：近距离有力，远距离力有限。"""
    apf = ImprovedArtificialPotentialField(k_comm=0.3, comm_range=10.0)
    pos = np.array([0.0, 0.0, 0.0], dtype=float)

    # 近距离有力
    f_near = apf._communication_constraint_force(pos, [np.array([8.5, 0.0, 0.0], dtype=float)])
    assert float(np.linalg.norm(f_near)) > 0, "接近通信范围边界应有约束力"

    # 远超通信范围时力 ≤ k_comm（门控饱和 + dist_ratio 钳制）
    f_far = apf._communication_constraint_force(pos, [np.array([50.0, 0.0, 0.0], dtype=float)])
    assert float(np.linalg.norm(f_far)) <= apf.k_comm + 1e-6, \
        f"远超通信范围时力不应超过 k_comm={apf.k_comm}"


def test_rotational_escape_tangent():
    """Rodrigues 旋转力场切向性验证：dot(F_rot, repulsion) ≈ 0。"""
    apf = ImprovedArtificialPotentialField(mu_escape=0.5)
    repulsion = np.array([1.0, 2.0, 0.5], dtype=float)
    pos = np.array([0.0, 0.0, 0.0], dtype=float)
    goal = np.array([10.0, 0.0, 0.0], dtype=float)

    f_rot = apf._compute_rotational_escape(repulsion, pos, goal)
    dot_val = float(np.dot(f_rot, repulsion))
    rep_norm = float(np.linalg.norm(repulsion))
    f_rot_norm = float(np.linalg.norm(f_rot))

    if f_rot_norm > 1e-6 and rep_norm > 1e-6:
        cos_angle = abs(dot_val) / (f_rot_norm * rep_norm)
        assert cos_angle < 0.1, \
            f"旋转力场应与斥力正交，cos={cos_angle:.4f}"


def test_apf_integration_no_crash():
    """APF 增强后基本调用不崩溃。"""
    apf = ImprovedArtificialPotentialField(
        adaptive_n_decay=True, k_comm=0.3, comm_range=10.0
    )
    obstacles = MockObstacles()
    pos = np.array([3.0, 3.0, 3.0], dtype=float)
    goal = np.array([10.0, 10.0, 10.0], dtype=float)
    others = [np.array([4.0, 3.0, 3.0], dtype=float)]

    acc = apf.compute_avoidance_acceleration(pos, goal, obstacles, others)
    assert acc.shape == (3,), "输出应为 (3,) 加速度向量"
    assert float(np.linalg.norm(acc)) <= apf.max_acc + 1e-6, \
        "总加速度应不超过 max_acc"


if __name__ == "__main__":
    tests = [
        ("n_decay 基本功能", test_adaptive_n_decay_basic),
        ("n_decay 缓存", test_adaptive_n_decay_cache),
        ("通信约束量纲", test_communication_constraint_force_dim),
        ("通信约束平滑", test_communication_constraint_smooth),
        ("Rodrigues 切向性", test_rotational_escape_tangent),
        ("APF 集成调用", test_apf_integration_no_crash),
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
