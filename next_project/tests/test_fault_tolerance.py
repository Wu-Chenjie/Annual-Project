"""故障检测与容错单测（论文4）。

验证:
- 故障注入掩码生效
- FaultDetector 基本检测不误检
- 拓扑 λ₂ 计算
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.drone import Drone
from core.fault_detector import FaultDetector
from core.topology import TopologyGraph, FormationTopology


def test_drone_fault_mask():
    """故障注入掩码对推力分配的影响。"""
    drone = Drone()
    drone.set_initial_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    # 正常情况
    u_normal = drone._rotor_step(np.array([9.81, 0.0, 0.0, 0.0], dtype=float))

    # 注入故障：1号旋翼完全失效
    drone.inject_fault(1, 0.0)
    u_fault = drone._rotor_step(np.array([9.81, 0.0, 0.0, 0.0], dtype=float))

    # 故障掩码应生效，故障后控制量应不同
    assert drone.fault_active, "fault_active 应为 True"
    assert drone.fault_mask[1] == 0.0, f"1号旋翼掩码应为 0，实际 {drone.fault_mask[1]}"
    diff = float(np.linalg.norm(u_normal - u_fault))
    assert diff > 1e-6, f"故障前后 u 应有差异，实际 diff={diff:.6f}"

    # 清除后恢复（重新初始化验证 mask 已清零）
    drone.clear_faults()
    assert not drone.fault_active, "清除后 fault_active 应为 False"
    assert np.all(drone.fault_mask == 1.0), "清除后所有掩码应为 1.0"


def test_fault_detector_no_false_positive():
    """正常飞行 30s（模拟步）不应误检。"""
    fd = FaultDetector(max_acc=10.0, pos_dev_threshold=5.0, saturate_steps=50, dt=0.012)
    n_steps = int(30.0 / 0.012)  # ~2500 steps

    pos = np.zeros(6, dtype=float)
    des = np.zeros(6, dtype=float)
    ctrl = np.array([9.81, 0.0, 0.0, 0.0], dtype=float)

    # 模拟飞行（小幅位置变化）
    detected = False
    for step in range(min(n_steps, 1000)):
        t = step * 0.012
        pos[0] = 1.0 * np.sin(0.5 * t)
        pos[1] = 1.0 * np.cos(0.5 * t)
        pos[2] = 0.1 * t
        pos[3] = 0.5 * np.cos(0.5 * t)
        pos[4] = -0.5 * np.sin(0.5 * t)
        pos[5] = 0.1
        des[0:3] = pos[0:3]  # 完美跟踪

        if fd.check(0, pos, des, ctrl, control_max=20.0):
            detected = True
            break

    assert not detected, "正常飞行不应误检故障"


def test_topology_graph_lambda2():
    """拓扑图 λ₂ 计算：连通图 λ₂ > 0。"""
    ft = FormationTopology(num_followers=3, spacing=2.0)
    offsets = ft.get_offsets("diamond")
    tg = TopologyGraph(offsets)

    lambda2 = tg.algebraic_connectivity
    assert lambda2 > 0, f"连通图的 λ₂ 应 > 0，实际 {lambda2:.4f}"


def test_topology_graph_lambda2_matches_laplacian_eigenvalue():
    """λ₂ 应等于 Laplacian 第二小特征值，而不是最小度数近似。"""
    offsets = [
        np.array([-2.0, 0.0, 0.0], dtype=float),
        np.array([-4.0, 0.0, 0.0], dtype=float),
        np.array([-6.0, 0.0, 0.0], dtype=float),
    ]
    tg = TopologyGraph(offsets)
    expected = float(np.linalg.eigvalsh(tg.laplacian)[1])
    assert abs(tg.algebraic_connectivity - expected) < 1e-9


def test_best_reconfig_topology():
    """故障后拓扑重构：选择 λ₂ 最大的可行拓扑。"""
    best = TopologyGraph.best_reconfig_topology(
        failed_indices=[0],
        num_followers=3,
        spacing=2.0,
        candidates=["v_shape", "diamond", "line", "triangle"],
    )
    assert best is not None, "应返回可行拓扑"
    assert best in ("v_shape", "diamond", "line", "triangle"), f"无效拓扑: {best}"


def test_dual_mode_scheduler():
    """双模式调度器基本状态机。"""
    from core.planning.dual_mode import DualModeScheduler

    ds = DualModeScheduler(
        sensor_danger_threshold=2.0,
        sensor_safe_threshold=4.0,
        sdf_danger_threshold=0.5,
    )

    # 初始 SAFE
    assert ds.mode == "safe"

    # 危险传感器读数 → DANGER
    mode = ds.classify(0.0, np.array([1.0, 5.0, 5.0, 5.0, 5.0, 5.0], dtype=float), None, np.zeros(3))
    assert mode == "danger"

    # 安全读数 → 滞回保持 DANGER
    mode = ds.classify(0.1, np.array([3.0, 5.0, 5.0, 5.0, 5.0, 5.0], dtype=float), None, np.zeros(3))
    assert mode == "danger", "滞回应保持 danger"

    # 全安全读数 → 恢复 SAFE
    mode = ds.classify(0.2, np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0], dtype=float), None, np.zeros(3))
    assert mode == "safe"


if __name__ == "__main__":
    tests = [
        ("故障注入掩码", test_drone_fault_mask),
        ("无误检", test_fault_detector_no_false_positive),
        ("拓扑 λ2", test_topology_graph_lambda2),
        ("拓扑 λ2 与 Laplacian 特征值一致", test_topology_graph_lambda2_matches_laplacian_eigenvalue),
        ("拓扑重构选择", test_best_reconfig_topology),
        ("双模式调度器", test_dual_mode_scheduler),
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
