"""路径规划抽象基类。

用途
----
定义规划器统一接口，提供路径平滑默认实现。

原理
----
Catmull-Rom 样条平滑：对 (K,3) 路径，在每个相邻段插入 M 个插值点，
使路径连续可微，消除栅格阶梯波形，便于控制器跟踪。

"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class PlannerError(Exception):
    """路径规划失败异常。"""
    pass


class Planner(ABC):
    """路径规划器抽象基类。"""

    @abstractmethod
    def plan(self, start: np.ndarray, goal: np.ndarray, grid, **kw) -> np.ndarray:
        """规划从 start 到 goal 的路径。

        参数
        ----
        start: (3,) 起点世界坐标。
        goal: (3,) 终点世界坐标。
        grid: OccupancyGrid 占据栅格。
        **kw: 算法特定参数。

        返回
        ----
        (K, 3) 路径航点数组。失败抛 PlannerError。
        """
        ...

    def smooth(self, path: np.ndarray, num_insert: int = 4, grid=None) -> np.ndarray:
        """Catmull-Rom 样条平滑。

        参数
        ----
        path: (K, 3) 原始路径。
        num_insert: 每段插入点数。
        grid: 可选占据栅格，子类可使用做碰撞校核。基类忽略此参数，
              仅为统一接口（避免子类签名漂移破坏 LSP）。

        返回
        ----
        (K', 3) 平滑后路径。

        端点处理
        --------
        采用镜像延拓 p0 = 2·p1 - p2、p3 = 2·p2 - p1，
        避免直接复制端点导致首末段切线幅值减半、曲率突变。
        """
        del grid  # 基类未使用
        path = np.asarray(path, dtype=float)
        if len(path) < 3:
            return path

        K = len(path)
        smoothed = [path[0]]
        for i in range(K - 1):
            # 镜像延拓代替端点复制，保持切线连续性
            p0 = path[i - 1] if i - 1 >= 0 else 2.0 * path[0] - path[1]
            p1 = path[i]
            p2 = path[i + 1]
            p3 = path[i + 2] if i + 2 < K else 2.0 * path[-1] - path[-2]
            for j in range(num_insert):
                t = (j + 1) / (num_insert + 1)
                tt = t * t
                ttt = tt * t
                pt = 0.5 * (
                    (2.0 * p1)
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * tt
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * ttt
                )
                smoothed.append(pt)
            smoothed.append(p2)
        return np.array(smoothed, dtype=float)
