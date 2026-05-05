"""编队拓扑管理。

用途
----
管理无人机编队的相对偏移量，并支持队形之间的平滑切换。

原理
----
设当前偏移为 Delta_src，目标偏移为 Delta_tgt，过渡时间为 T。
在 t in [0, T] 内使用余弦插值：

    alpha(t) = 0.5 * (1 - cos(pi * t / T))
    Delta(t) = (1 - alpha) * Delta_src + alpha * Delta_tgt

这种方式在起点和终点均具有零速度特性，可减小切换冲击。


"""

from __future__ import annotations

import numpy as np


class FormationTopology:
    """动态编队拓扑管理器。"""

    PREDEFINED_FORMATIONS = ("v_shape", "diamond", "line", "triangle")

    def __init__(self, num_followers: int = 3, spacing: float = 2.0, arm_length: float = 0.2):
        if num_followers < 1:
            raise ValueError("num_followers must be >= 1")
        if spacing <= 0:
            raise ValueError("spacing must be positive")
        self.num_followers = int(num_followers)
        self.spacing = float(spacing)
        self.arm_length = float(arm_length)
        self.current_formation = "v_shape"
        self._current_offsets = self._compute_offsets("v_shape")
        self.custom_offsets = None
        self._switching = False
        self._switch_start_time = 0.0
        self._transition_time = 0.0
        self._source_offsets = []
        self._target_offsets = []
        self._target_formation = ""

    def _compute_v_shape(self):
        offsets = []
        for i in range(self.num_followers):
            rank = (i // 2) + 1
            side = -1 if i % 2 == 0 else 1
            offsets.append(np.array([-rank * self.spacing, side * rank * self.spacing, 0.0], dtype=float))
        return offsets

    def _compute_diamond(self):
        offsets = [
            np.array([-self.spacing, -self.spacing, 0.0], dtype=float),
            np.array([-self.spacing, self.spacing, 0.0], dtype=float),
            np.array([-2.0 * self.spacing, 0.0, 0.0], dtype=float),
        ]
        while len(offsets) < self.num_followers:
            extra_rank = len(offsets) - 2
            offsets.append(np.array([-2.0 * self.spacing - extra_rank * self.spacing, 0.0, 0.0], dtype=float))
        return offsets[:self.num_followers]

    def _compute_line(self):
        return [np.array([-(i + 1) * self.spacing, 0.0, 0.0], dtype=float) for i in range(self.num_followers)]

    def _compute_triangle(self):
        offsets = []
        row_height = self.spacing * np.sqrt(3.0) / 2.0
        placed = 0
        row = 1
        while placed < self.num_followers:
            num_in_row = row + 1
            row_y_start = -row * self.spacing / 2.0
            for col in range(num_in_row):
                if placed >= self.num_followers:
                    break
                dx = -row * row_height
                dy = row_y_start + col * self.spacing
                offsets.append(np.array([dx, dy, 0.0], dtype=float))
                placed += 1
            row += 1
        return offsets

    def _compute_offsets(self, formation: str):
        if formation == "v_shape":
            return self._compute_v_shape()
        if formation == "diamond":
            return self._compute_diamond()
        if formation == "line":
            return self._compute_line()
        if formation == "triangle":
            return self._compute_triangle()
        if formation == "custom":
            if self.custom_offsets is None:
                raise ValueError("custom formation requires set_custom_offsets first")
            if len(self.custom_offsets) != self.num_followers:
                raise ValueError("custom offsets count does not match followers")
            return [off.copy() for off in self.custom_offsets]
        raise ValueError(f"unknown formation: {formation}")

    def get_offsets(self, formation: str):
        return self._compute_offsets(formation)

    def set_custom_offsets(self, offsets) -> None:
        if len(offsets) != self.num_followers:
            raise ValueError("offset count does not match followers")
        self.custom_offsets = [np.array(off, dtype=float).flatten()[:3] for off in offsets]

    def switch_formation(self, target_formation: str, transition_time: float = 5.0) -> None:
        if transition_time <= 0:
            raise ValueError("transition_time must be positive")
        target_offsets = self._compute_offsets(target_formation)
        self._source_offsets = [off.copy() for off in self._current_offsets]
        self._target_offsets = target_offsets
        self._target_formation = target_formation
        self._transition_time = float(transition_time)
        self._switch_start_time = 0.0
        self._switching = True

    def get_current_offsets(self, elapsed_time: float):
        if not self._switching:
            return [off.copy() for off in self._current_offsets]
        t = elapsed_time - self._switch_start_time
        if t >= self._transition_time:
            self._current_offsets = [off.copy() for off in self._target_offsets]
            self.current_formation = self._target_formation
            self._switching = False
            return [off.copy() for off in self._current_offsets]
        alpha = 0.5 * (1.0 - np.cos(np.pi * t / self._transition_time))
        return [(1.0 - alpha) * src + alpha * tgt for src, tgt in zip(self._source_offsets, self._target_offsets)]

    def envelope_radius(self, formation_type: str | None = None) -> float:
        """返回当前/指定队形下编队完整包络半径（含臂长）。

        用途
        ----
        供路径规划器计算膨胀半径：r_inflate = envelope_radius + safety_margin。
        envelope_radius 已包含臂长，调用方不应再次叠加 arm_length。

        原理
        ----
        遍历队形偏移量，取最大范数并加上臂长。
        """
        offsets = self.get_offsets(formation_type or self.current_formation)
        max_dist = max(float(np.linalg.norm(off)) for off in offsets) if offsets else 0.0
        return max_dist + self.arm_length

    @property
    def is_switching(self) -> bool:
        return self._switching

    # ------------------------------------------------------------------
    # 论文4: 容错拓扑重构
    # ------------------------------------------------------------------

    def fault_reconfigure(self, failed_indices: list[int], transition_time: float = 3.0) -> str | None:
        """故障后自动选择最佳拓扑并开始切换。

        failed_indices: 故障从机索引列表（0-based）。
        返回选中的拓扑类型，或 None（无可用拓扑）。
        """
        candidates = ["v_shape", "diamond", "line", "triangle"]
        best_topo = TopologyGraph.best_reconfig_topology(
            failed_indices, self.num_followers, self.spacing, candidates
        )
        if best_topo is not None:
            self.switch_formation(best_topo, transition_time)
        return best_topo

    # ------------------------------------------------------------------
    # 论文2: 队形自动收缩
    # ------------------------------------------------------------------

    def auto_shrink(self, channel_width: float, envelope: float | None = None) -> bool:
        """基于通道宽度自动触发队形缩放。

        若 channel_width < 2*envelope → 缩为 line 队形。
        返回 True 表示已触发切换。
        """
        if envelope is None:
            envelope = self.envelope_radius(self.current_formation)
        if channel_width < 2.0 * envelope and self.current_formation != "line":
            self.switch_formation("line", transition_time=1.5)
            return True
        elif channel_width > 3.0 * envelope and self.current_formation == "line":
            self.switch_formation("diamond", transition_time=2.0)
            return True
        return False


class TopologyGraph:
    """编队图论拓扑模型（论文4 L3）。

    用途
    ----
    用图 Laplacian 矩阵描述编队拓扑关系，提取代数连通度 λ₂，
    作为拓扑鲁棒性度量和故障重构选择的依据。
    """

    def __init__(self, offsets: list[np.ndarray]):
        self.n = len(offsets) + 1  # followers + leader
        self._offsets = [np.asarray(o, dtype=float) for o in offsets]
        self._laplacian = self._build_laplacian()

    def _build_laplacian(self) -> np.ndarray:
        """从几何偏移构建 Laplacian 矩阵。

        节点 0 = 领航机，节点 1..n = 从机。
        a_ij = exp(-||offset_i - offset_j||² / σ²)
        """
        n = self.n
        sigma = 2.0  # 距离缩放参数
        positions = [np.zeros(3, dtype=float)] + self._offsets
        A = np.zeros((n, n), dtype=float)
        for i in range(n):
            for j in range(i + 1, n):
                d2 = float(np.sum((positions[i] - positions[j]) ** 2))
                w = np.exp(-d2 / (sigma ** 2))
                A[i, j] = w
                A[j, i] = w
        D = np.diag(np.sum(A, axis=1))
        return D - A

    @property
    def algebraic_connectivity(self) -> float:
        """λ₂ = Laplacian 的第二小特征值。越大表示拓扑连通性越强。"""
        eigenvals = np.linalg.eigvalsh(self._laplacian)
        return float(eigenvals[1])

    @property
    def laplacian(self) -> np.ndarray:
        return self._laplacian

    @staticmethod
    def best_reconfig_topology(
        failed_indices: list[int],
        num_followers: int,
        spacing: float,
        candidates: list[str] = ("v_shape", "diamond", "line", "triangle"),
    ) -> str | None:
        """故障后选择 λ₂ 最大的可行拓扑。

        1. 排除含故障机的候选配置（剩余健康的从机数量要能组成该队形）
        2. 对每个候选拓扑构建 Laplacian
        3. 返回 λ₂ 最大的拓扑类型
        """
        healthy = num_followers - len(failed_indices)
        if healthy < 1:
            return None

        best = candidates[0]
        best_lambda2 = -1.0

        for topo in candidates:
            # 计算该队形对健康数量的偏移
            offsets = TopologyGraph._compute_offsets_for(topo, healthy, spacing)
            if offsets is None or len(offsets) != healthy:
                continue
            tg = TopologyGraph(offsets)
            l2 = tg.algebraic_connectivity
            if l2 > best_lambda2:
                best_lambda2 = l2
                best = topo

        return best

    @staticmethod
    def _compute_offsets_for(formation: str, n: int, spacing: float) -> list[np.ndarray] | None:
        """为指定队形和从机数计算偏移量（静态辅助方法）。"""
        try:
            ft = FormationTopology(num_followers=n, spacing=spacing)
            return ft._compute_offsets(formation)
        except Exception:
            return None
