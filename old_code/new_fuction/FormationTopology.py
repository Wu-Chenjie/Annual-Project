"""
FormationTopology.py -- 动态编队拓扑管理模块

管理多种预定义编队队形（V形、菱形、线形、三角形、自定义），
支持队形之间的余弦插值平滑切换。

所有偏移量相对于领航机位置 (0,0,0)，z 方向偏移为 0（同高度编队）。
"""

import numpy as np


class FormationTopology:
    """动态编队拓扑管理器。

    管理从机相对于领航机的编队偏移量，支持多种预定义队形和自定义队形，
    并提供基于余弦插值的平滑队形切换功能。

    参数
    ----------
    num_followers : int
        从机数量（不含领航机）。
    spacing : float
        无人机基础间距（米），默认 2.0。

    属性
    ----------
    num_followers : int
        从机数量。
    spacing : float
        基础间距。
    current_formation : str
        当前队形名称。
    custom_offsets : list[np.ndarray] | None
        用户自定义偏移量列表。

    示例
    --------
    >>> topo = FormationTopology(num_followers=3, spacing=2.0)
    >>> offsets = topo.get_offsets("v_shape")       # 获取 V 形偏移量
    >>> topo.switch_formation("diamond", transition_time=5.0)  # 启动切换
    >>> offsets = topo.get_current_offsets(2.5)      # 过渡中获取插值偏移量
    """

    # 支持的预定义队形名称
    PREDEFINED_FORMATIONS = ("v_shape", "diamond", "line", "triangle")

    def __init__(self, num_followers: int = 3, spacing: float = 2.0):
        if num_followers < 1:
            raise ValueError("从机数量 num_followers 必须 >= 1")
        if spacing <= 0:
            raise ValueError("间距 spacing 必须为正数")

        self.num_followers = num_followers
        self.spacing = spacing

        # 当前队形状态
        self.current_formation: str = "v_shape"
        self._current_offsets: list[np.ndarray] = self._compute_offsets("v_shape")

        # 自定义偏移量存储
        self.custom_offsets: list[np.ndarray] | None = None

        # 队形切换状态
        self._switching: bool = False
        self._switch_start_time: float = 0.0
        self._transition_time: float = 0.0
        self._source_offsets: list[np.ndarray] = []
        self._target_offsets: list[np.ndarray] = []
        self._target_formation: str = ""

    # ------------------------------------------------------------------
    #  预定义队形生成
    # ------------------------------------------------------------------

    def _compute_v_shape(self) -> list[np.ndarray]:
        """计算 V 形编队偏移量。

        领航机在最前方，从机向后方两侧展开。
        奇数索引分配到左侧，偶数索引分配到右侧。

        返回
        ------
        list[np.ndarray]
            每个从机相对于领航机的 3D 偏移量。
        """
        offsets = []
        for i in range(self.num_followers):
            # 第 i 架从机：交替分配左右两侧
            rank = (i // 2) + 1          # 第几排（从 1 开始）
            side = -1 if (i % 2 == 0) else 1  # 左(-1) / 右(+1)

            dx = -rank * self.spacing    # 向后
            dy = side * rank * self.spacing  # 左右展开
            offsets.append(np.array([dx, dy, 0.0]))
        return offsets

    def _compute_diamond(self) -> list[np.ndarray]:
        """计算菱形编队偏移量。

        领航机在最前端，从机依次排列形成菱形轮廓：
        - 第 1 架：左后方
        - 第 2 架：右后方
        - 第 3 架：正后方（菱形尾端）
        - 第 4+ 架：沿菱形轮廓等间距扩展

        返回
        ------
        list[np.ndarray]
            每个从机相对于领航机的 3D 偏移量。
        """
        offsets = []
        # 菱形的基本 4 个位置（不含领航机）
        diamond_template = [
            np.array([-self.spacing, -self.spacing, 0.0]),   # 左后
            np.array([-self.spacing,  self.spacing, 0.0]),   # 右后
            np.array([-2 * self.spacing, 0.0, 0.0]),         # 正后（尾端）
        ]

        for i in range(self.num_followers):
            if i < len(diamond_template):
                offsets.append(diamond_template[i].copy())
            else:
                # 超出基本菱形的从机：在尾端后方依次排列
                extra_rank = i - len(diamond_template) + 1
                offsets.append(np.array([
                    -2 * self.spacing - extra_rank * self.spacing,
                    0.0,
                    0.0
                ]))
        return offsets

    def _compute_line(self) -> list[np.ndarray]:
        """计算线形编队偏移量。

        所有从机沿 x 轴负方向（领航机后方）等间距排列。

        返回
        ------
        list[np.ndarray]
            每个从机相对于领航机的 3D 偏移量。
        """
        offsets = []
        for i in range(self.num_followers):
            dx = -(i + 1) * self.spacing
            offsets.append(np.array([dx, 0.0, 0.0]))
        return offsets

    def _compute_triangle(self) -> list[np.ndarray]:
        """计算等边三角形编队偏移量。

        领航机位于三角形顶点（最前方），从机沿后续行逐行排列，
        每行的无人机数量递增，行间距为 spacing * sqrt(3)/2（等边三角形行高）。

        返回
        ------
        list[np.ndarray]
            每个从机相对于领航机的 3D 偏移量。
        """
        offsets = []
        row_height = self.spacing * np.sqrt(3) / 2  # 等边三角形行高
        placed = 0
        row = 1  # 从第 1 行开始（第 0 行是领航机）

        while placed < self.num_followers:
            # 当前行放置 (row + 1) 架，但领航机已占第 0 行的 1 个位置
            num_in_row = row + 1
            row_y_start = -row * self.spacing / 2  # 该行最左侧 y 坐标

            for col in range(num_in_row):
                if placed >= self.num_followers:
                    break
                dx = -row * row_height
                dy = row_y_start + col * self.spacing
                offsets.append(np.array([dx, dy, 0.0]))
                placed += 1
            row += 1

        return offsets

    def _compute_offsets(self, formation: str) -> list[np.ndarray]:
        """根据队形名称计算偏移量。

        参数
        ----------
        formation : str
            队形名称，支持 "v_shape", "diamond", "line", "triangle", "custom"。

        返回
        ------
        list[np.ndarray]
            从机偏移量列表。

        异常
        ------
        ValueError
            队形名称未知或自定义偏移量未设置。
        """
        if formation == "v_shape":
            return self._compute_v_shape()
        elif formation == "diamond":
            return self._compute_diamond()
        elif formation == "line":
            return self._compute_line()
        elif formation == "triangle":
            return self._compute_triangle()
        elif formation == "custom":
            if self.custom_offsets is None:
                raise ValueError(
                    "自定义队形需先调用 set_custom_offsets() 设置偏移量"
                )
            if len(self.custom_offsets) != self.num_followers:
                raise ValueError(
                    f"自定义偏移量数量 ({len(self.custom_offsets)}) "
                    f"与从机数量 ({self.num_followers}) 不匹配"
                )
            return [off.copy() for off in self.custom_offsets]
        else:
            raise ValueError(
                f"未知队形 '{formation}'，"
                f"支持: {self.PREDEFINED_FORMATIONS + ('custom',)}"
            )

    # ------------------------------------------------------------------
    #  公开接口
    # ------------------------------------------------------------------

    def get_offsets(self, formation: str) -> list[np.ndarray]:
        """获取指定队形的从机偏移量（不改变当前状态）。

        参数
        ----------
        formation : str
            队形名称。

        返回
        ------
        list[np.ndarray]
            各从机相对于领航机的 3D 偏移量列表，长度为 num_followers。
        """
        return self._compute_offsets(formation)

    def set_custom_offsets(self, offsets: list[np.ndarray]) -> None:
        """设置自定义队形偏移量。

        参数
        ----------
        offsets : list[np.ndarray]
            用户定义的偏移量列表，长度需等于 num_followers，
            每个元素为 shape=(3,) 的 numpy 数组。

        异常
        ------
        ValueError
            偏移量数量或维度不正确。
        """
        if len(offsets) != self.num_followers:
            raise ValueError(
                f"偏移量数量 ({len(offsets)}) 与从机数量 ({self.num_followers}) 不匹配"
            )
        self.custom_offsets = [np.array(off, dtype=float).flatten()[:3] for off in offsets]

    def switch_formation(self, target_formation: str, transition_time: float = 5.0) -> None:
        """启动队形平滑切换。

        使用余弦插值在 transition_time 秒内从当前队形过渡到目标队形。
        切换期间通过 get_current_offsets(t) 获取插值偏移量。

        参数
        ----------
        target_formation : str
            目标队形名称。
        transition_time : float
            过渡时间（秒），默认 5.0。必须为正数。

        异常
        ------
        ValueError
            过渡时间非正或目标队形无效。
        """
        if transition_time <= 0:
            raise ValueError("过渡时间 transition_time 必须为正数")

        # 预先计算目标偏移量（同时验证队形名称有效性）
        target_offsets = self._compute_offsets(target_formation)

        # 记录切换起始状态
        self._source_offsets = [off.copy() for off in self._current_offsets]
        self._target_offsets = target_offsets
        self._target_formation = target_formation
        self._transition_time = transition_time
        self._switch_start_time = 0.0  # 相对时间从 0 开始
        self._switching = True

    def get_current_offsets(self, elapsed_time: float) -> list[np.ndarray]:
        """获取当前时刻的从机偏移量（考虑队形切换过渡）。

        如果正处于队形切换过渡期间，返回余弦插值后的偏移量；
        否则直接返回当前队形的偏移量。

        余弦插值公式：
            alpha = 0.5 * (1 - cos(pi * t / T))
            offset = (1 - alpha) * source + alpha * target

        参数
        ----------
        elapsed_time : float
            自调用 switch_formation() 以来经过的时间（秒）。
            如果未在切换中，此参数被忽略。

        返回
        ------
        list[np.ndarray]
            当前时刻各从机的 3D 偏移量列表。
        """
        if not self._switching:
            return [off.copy() for off in self._current_offsets]

        # 计算过渡进度
        t = elapsed_time - self._switch_start_time
        if t >= self._transition_time:
            # 切换完成
            self._current_offsets = [off.copy() for off in self._target_offsets]
            self.current_formation = self._target_formation
            self._switching = False
            return [off.copy() for off in self._current_offsets]

        # 余弦插值：alpha 从 0 平滑过渡到 1
        alpha = 0.5 * (1.0 - np.cos(np.pi * t / self._transition_time))

        interpolated = []
        for src, tgt in zip(self._source_offsets, self._target_offsets):
            interpolated.append((1.0 - alpha) * src + alpha * tgt)
        return interpolated

    @property
    def is_switching(self) -> bool:
        """当前是否正处于队形切换过渡中。"""
        return self._switching

    @property
    def available_formations(self) -> tuple[str, ...]:
        """返回所有可用队形名称。"""
        formations = list(self.PREDEFINED_FORMATIONS)
        if self.custom_offsets is not None:
            formations.append("custom")
        return tuple(formations)

    def __repr__(self) -> str:
        return (
            f"FormationTopology(num_followers={self.num_followers}, "
            f"spacing={self.spacing}, "
            f"current_formation='{self.current_formation}', "
            f"switching={self._switching})"
        )


# ----------------------------------------------------------------------
#  模块自测
# ----------------------------------------------------------------------
if __name__ == "__main__":
    print("=== FormationTopology 模块自测 ===\n")

    topo = FormationTopology(num_followers=4, spacing=2.0)

    # 测试各种预定义队形
    for name in ("v_shape", "diamond", "line", "triangle"):
        offsets = topo.get_offsets(name)
        print(f"[{name}] 偏移量:")
        for i, off in enumerate(offsets):
            print(f"  从机 {i}: {off}")
        print()

    # 测试自定义队形
    custom = [np.array([1.0, 2.0, 0.0]),
              np.array([-1.0, 2.0, 0.0]),
              np.array([0.0, -2.0, 0.0]),
              np.array([0.0, 3.0, 0.0])]
    topo.set_custom_offsets(custom)
    offsets = topo.get_offsets("custom")
    print("[custom] 偏移量:")
    for i, off in enumerate(offsets):
        print(f"  从机 {i}: {off}")
    print()

    # 测试平滑切换
    print("--- 平滑切换测试: v_shape -> diamond (5s) ---")
    topo2 = FormationTopology(num_followers=3, spacing=2.0)
    topo2.switch_formation("diamond", transition_time=5.0)

    for t in [0.0, 1.0, 2.5, 4.0, 5.0, 6.0]:
        offsets = topo2.get_current_offsets(t)
        status = "过渡中" if topo2.is_switching else "已完成"
        print(f"  t={t:.1f}s [{status}]: 从机0={offsets[0]}")

    print("\n=== 自测完成 ===")
