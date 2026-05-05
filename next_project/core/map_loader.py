"""室内地图导入。

用途
----
从 JSON 原语描述或 NPZ 占据栅格文件加载障碍物地图，
返回 ObstacleField 或 OccupancyGrid 供仿真使用。

原理
----
JSON 格式：{"bounds": [[xmin,ymin,zmin],[xmax,ymax,zmax]],
"obstacles": [{"type":"aabb","min":[...],"max":[...]}, ...]}

NPZ 格式：{data: uint8 3D array, origin: (3,), resolution: float}

"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .obstacles import AABB, Sphere, Cylinder, ObstacleField, OccupancyGrid


def load_from_json(filepath: str) -> tuple[ObstacleField, np.ndarray]:
    """从 JSON 原语描述加载障碍物。

    返回 (ObstacleField, bounds) 其中 bounds shape=(2,3)。
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"地图文件不存在: {filepath}")

    with open(path, encoding="utf-8") as f:
        data = json.load(f)

    bounds = np.array(data["bounds"], dtype=float)
    field = ObstacleField()

    for obs in data.get("obstacles", []):
        obs_type = obs["type"]
        if obs_type == "aabb":
            field.add(AABB(np.array(obs["min"], dtype=float), np.array(obs["max"], dtype=float)))
        elif obs_type == "sphere":
            field.add(Sphere(np.array(obs["center"], dtype=float), float(obs["radius"])))
        elif obs_type == "cylinder":
            field.add(Cylinder(
                np.array(obs["center_xy"], dtype=float),
                float(obs["radius"]),
                (float(obs["z_range"][0]), float(obs["z_range"][1])),
            ))
        else:
            raise ValueError(f"未知障碍物类型: {obs_type}")

    return field, bounds


def load_from_npz(filepath: str) -> OccupancyGrid:
    """从 NPZ 占据栅格文件加载。"""
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"地图文件不存在: {filepath}")

    npz = np.load(path)
    return OccupancyGrid(
        origin=np.array(npz["origin"], dtype=float),
        resolution=float(npz["resolution"]),
        shape=tuple(npz["shape"]),
        data=npz["data"].astype(np.uint8),
    )
