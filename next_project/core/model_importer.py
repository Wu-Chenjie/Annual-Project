"""3D 模型文件导入：PLY / OBJ / STL → 障碍物场。

将 web/server.py 的模型解析与体素化逻辑移植到 Python 仿真端，
对外暴露 import_model() 作为主入口，返回 ObstacleField + bounds。
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from pathlib import Path
from typing import Any

import numpy as np

from .obstacles import AABB, ObstacleField

PLY_SCALARS: dict[str, tuple[str, int]] = {
    "char": ("b", 1),
    "int8": ("b", 1),
    "uchar": ("B", 1),
    "uint8": ("B", 1),
    "short": ("h", 2),
    "int16": ("h", 2),
    "ushort": ("H", 2),
    "uint16": ("H", 2),
    "int": ("i", 4),
    "int32": ("i", 4),
    "uint": ("I", 4),
    "uint32": ("I", 4),
    "float": ("f", 4),
    "float32": ("f", 4),
    "double": ("d", 8),
    "float64": ("d", 8),
}

Vec3 = tuple[float, float, float]
Triangle = tuple[int, int, int]


@dataclass(frozen=True)
class ImportedMesh:
    vertices: list[Vec3]
    faces: list[Triangle]
    source_name: str


def _vec_cell(point: Vec3, voxel: float) -> tuple[int, int, int]:
    return (
        int(point[0] // voxel),
        int(point[1] // voxel),
        int(point[2] // voxel),
    )


# ============================================================
# 格式解析器
# ============================================================


def _parse_obj(text: str) -> tuple[list[Vec3], list[Triangle]]:
    vertices: list[Vec3] = []
    triangles: list[Triangle] = []
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("v "):
            parts = line.split()
            if len(parts) >= 4:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("f "):
            refs: list[int] = []
            for token in line.split()[1:]:
                value = token.split("/")[0]
                if not value:
                    continue
                index = int(value)
                refs.append(index - 1 if index > 0 else len(vertices) + index)
            for i in range(1, len(refs) - 1):
                triangles.append((refs[0], refs[i], refs[i + 1]))
    return vertices, triangles


def _parse_stl(data: bytes) -> tuple[list[Vec3], list[Triangle]]:
    vertices: list[Vec3] = []
    triangles: list[Triangle] = []

    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + tri_count * 50 == len(data):
            offset = 84
            for _ in range(tri_count):
                base = len(vertices)
                for j in range(3):
                    vertices.append(struct.unpack_from("<fff", data, offset + 12 + j * 12))
                triangles.append((base, base + 1, base + 2))
                offset += 50
            return vertices, triangles

    text = data.decode("utf-8", errors="ignore")
    current: list[int] = []
    for raw in text.splitlines():
        parts = raw.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            current.append(len(vertices))
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(current) == 3:
                triangles.append((current[0], current[1], current[2]))
                current = []
    return vertices, triangles


def _parse_ply(data: bytes) -> tuple[list[Vec3], list[Triangle]]:
    marker = b"end_header"
    header_end = data.find(marker)
    if header_end < 0:
        raise ValueError("PLY header missing end_header")
    header_stop = data.find(b"\n", header_end)
    if header_stop < 0:
        header_stop = header_end + len(marker)
    header = data[:header_stop].decode("utf-8", errors="ignore")
    lines = header.splitlines()
    if not lines or lines[0].strip() != "ply":
        raise ValueError("Invalid PLY file")

    vertex_count = 0
    face_count = 0
    fmt = "ascii"
    vertex_properties: list[tuple[str, str]] = []
    in_vertex = False
    in_face = False
    for raw in lines:
        parts = raw.strip().split()
        if len(parts) >= 3 and parts[0] == "format":
            fmt = parts[1]
        if len(parts) == 3 and parts[:2] == ["element", "vertex"]:
            vertex_count = int(parts[2])
            in_vertex = True
            in_face = False
        elif len(parts) == 3 and parts[:2] == ["element", "face"]:
            face_count = int(parts[2])
            in_vertex = False
            in_face = True
        elif len(parts) == 3 and parts[0] == "property" and in_vertex:
            vertex_properties.append((parts[2], parts[1]))
        elif parts and parts[0] == "element":
            in_vertex = False
            in_face = False

    if fmt == "ascii":
        return _parse_ascii_ply(data[header_stop + 1:].decode("utf-8", errors="ignore"), vertex_count, face_count)
    if fmt != "binary_little_endian":
        raise ValueError(f"Unsupported PLY format: {fmt}")
    return _parse_binary_little_ply(data[header_stop + 1:], vertex_count, face_count, vertex_properties)


def _parse_ascii_ply(body: str, vertex_count: int, face_count: int) -> tuple[list[Vec3], list[Triangle]]:
    lines = body.splitlines()
    vertices: list[Vec3] = []
    for raw in lines[:vertex_count]:
        parts = raw.split()
        if len(parts) >= 3:
            vertices.append((float(parts[0]), float(parts[1]), float(parts[2])))
    triangles: list[Triangle] = []
    for raw in lines[vertex_count:vertex_count + face_count]:
        parts = raw.split()
        if not parts:
            continue
        count = int(parts[0])
        refs = [int(value) for value in parts[1:1 + count]]
        for i in range(1, len(refs) - 1):
            triangles.append((refs[0], refs[i], refs[i + 1]))
    return vertices, triangles


def _parse_binary_little_ply(
    body: bytes,
    vertex_count: int,
    face_count: int,
    vertex_properties: list[tuple[str, str]],
) -> tuple[list[Vec3], list[Triangle]]:
    if not vertex_properties:
        vertex_properties = [("x", "float"), ("y", "float"), ("z", "float")]
    offset = 0
    vertices: list[Vec3] = []
    for _ in range(vertex_count):
        values: dict[str, float] = {}
        for name, typ in vertex_properties:
            info = PLY_SCALARS.get(typ)
            if not info:
                raise ValueError(f"Unsupported PLY vertex property type: {typ}")
            code, size = info
            if offset + size > len(body):
                raise ValueError("Unexpected end of binary PLY vertex data")
            values[name] = float(struct.unpack_from("<" + code, body, offset)[0])
            offset += size
        vertices.append((values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0)))

    triangles: list[Triangle] = []
    for _ in range(face_count):
        if offset >= len(body):
            break
        count = body[offset]
        offset += 1
        refs: list[int] = []
        for _ in range(count):
            if offset + 4 > len(body):
                break
            refs.append(struct.unpack_from("<i", body, offset)[0])
            offset += 4
        for i in range(1, len(refs) - 1):
            triangles.append((refs[0], refs[i], refs[i + 1]))
    return vertices, triangles


def _parse_model(filepath: str) -> tuple[list[Vec3], list[Triangle]]:
    """根据文件扩展名分发到对应的解析器。"""
    ext = Path(filepath).suffix.lower()
    with open(filepath, "rb") as f:
        data = f.read()

    if ext == ".obj":
        return _parse_obj(data.decode("utf-8", errors="ignore"))
    if ext == ".stl":
        return _parse_stl(data)
    if ext == ".ply":
        return _parse_ply(data)
    raise ValueError(f"不支持的模型格式: {ext}，仅支持 .obj / .stl / .ply")


# ============================================================
# 三角面采样
# ============================================================


def parse_model_bytes(data: bytes, *, filename: str) -> ImportedMesh:
    """Parse OBJ/STL/PLY bytes into an ImportedMesh."""
    ext = Path(filename or "").suffix.lower()
    if ext == ".obj":
        vertices, faces = _parse_obj(data.decode("utf-8", errors="ignore"))
    elif ext == ".stl":
        vertices, faces = _parse_stl(data)
    elif ext == ".ply":
        vertices, faces = _parse_ply(data)
    else:
        raise ValueError(f"Unsupported model format: {ext}. Supported: .obj / .stl / .ply")
    return ImportedMesh(vertices=vertices, faces=faces, source_name=filename)


def _sample_triangle(a: Vec3, b: Vec3, c: Vec3, voxel: float) -> list[Vec3]:
    """对三角面进行重心坐标采样，保证体素化不遗漏薄面。"""
    edge = max(
        ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5,
        ((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2 + (a[2] - c[2]) ** 2) ** 0.5,
        ((b[0] - c[0]) ** 2 + (b[1] - c[1]) ** 2 + (b[2] - c[2]) ** 2) ** 0.5,
    )
    steps = max(1, min(48, int(edge / max(voxel * 0.5, 1e-6)) + 1))
    out: list[Vec3] = []
    for i in range(steps + 1):
        for j in range(steps + 1 - i):
            u = i / steps
            v = j / steps
            w = 1.0 - u - v
            out.append((
                a[0] * w + b[0] * u + c[0] * v,
                a[1] * w + b[1] * u + c[1] * v,
                a[2] * w + b[2] * u + c[2] * v,
            ))
    return out


# ============================================================
# 体素化 → AABB 障碍物
# ============================================================


def _model_to_obstacle_field(
    vertices: list[Vec3],
    triangles: list[Triangle],
    *,
    voxel_size: float,
    scale: float,
    padding: float,
    max_obstacles: int,
) -> tuple[ObstacleField, np.ndarray]:
    """将顶点/三角面体素化为 AABB 障碍物，返回 (ObstacleField, bounds)。"""
    if not vertices:
        raise ValueError("Model contains no vertices")
    if voxel_size <= 0 or scale <= 0:
        raise ValueError("voxel_size and scale must be positive")

    scaled = [(x * scale, y * scale, z * scale) for x, y, z in vertices]
    min_x = min(p[0] for p in scaled)
    min_y = min(p[1] for p in scaled)
    min_z = min(p[2] for p in scaled)
    shifted: list[Vec3] = [(x - min_x + padding, y - min_y + padding, z - min_z) for x, y, z in scaled]

    occupied: set[tuple[int, int, int]] = {_vec_cell(point, voxel_size) for point in shifted}
    for ia, ib, ic in triangles:
        if ia >= len(shifted) or ib >= len(shifted) or ic >= len(shifted):
            continue
        for point in _sample_triangle(shifted[ia], shifted[ib], shifted[ic], voxel_size):
            occupied.add(_vec_cell(point, voxel_size))

    runs: list[tuple[int, int, int, int]] = []
    by_yz: dict[tuple[int, int], list[int]] = {}
    for ix, iy, iz in occupied:
        by_yz.setdefault((iy, iz), []).append(ix)
    for (iy, iz), xs in by_yz.items():
        ordered = sorted(set(xs))
        start = prev = ordered[0]
        for value in ordered[1:]:
            if value == prev + 1:
                prev = value
            else:
                runs.append((start, prev, iy, iz))
                start = prev = value
        runs.append((start, prev, iy, iz))

    if len(runs) > max_obstacles:
        raise ValueError(
            f"Converted model produced {len(runs)} obstacles. "
            f"Increase voxel_size (current: {voxel_size}) or max_obstacles (current: {max_obstacles})."
        )

    field = ObstacleField()
    for x0, x1, iy, iz in runs:
        field.add(AABB(
            np.array([x0 * voxel_size, iy * voxel_size, iz * voxel_size], dtype=float),
            np.array([(x1 + 1) * voxel_size, (iy + 1) * voxel_size, (iz + 1) * voxel_size], dtype=float),
        ))

    max_x = max(p[0] for p in shifted) + padding
    max_y = max(p[1] for p in shifted) + padding
    max_z = max(p[2] for p in shifted) + max(voxel_size, padding * 0.25)
    bounds = np.array([[0.0, 0.0, 0.0], [max_x, max_y, max_z]], dtype=float)

    return field, bounds


def model_to_map_json(
    mesh: ImportedMesh,
    *,
    voxel_size: float = 0.4,
    scale: float = 1.0,
    padding: float = 0.5,
    max_obstacles: int = 500,
    resolution: float | None = None,
) -> dict[str, Any]:
    """Convert an ImportedMesh to the Web/map-loader JSON obstacle format."""
    voxel = float(resolution if resolution is not None else voxel_size)
    field, bounds = _model_to_obstacle_field(
        mesh.vertices,
        mesh.faces,
        voxel_size=voxel,
        scale=scale,
        padding=padding,
        max_obstacles=max_obstacles,
    )
    obstacles = []
    for obs in field:
        if not isinstance(obs, AABB):
            continue
        obstacles.append({
            "type": "aabb",
            "min": [round(float(value), 4) for value in obs.min_corner.tolist()],
            "max": [round(float(value), 4) for value in obs.max_corner.tolist()],
        })
    return {
        "bounds": [
            [round(float(value), 4) for value in bounds[0].tolist()],
            [round(float(value), 4) for value in bounds[1].tolist()],
        ],
        "description": f"Imported from 3D model {mesh.source_name}; voxel_size={voxel}, scale={scale}",
        "obstacles": obstacles,
    }


# ============================================================
# 公共接口
# ============================================================


def import_model(
    filepath: str,
    *,
    voxel_size: float = 0.4,
    scale: float = 1.0,
    padding: float = 0.5,
    max_obstacles: int = 500,
) -> tuple[ObstacleField, np.ndarray]:
    """从 3D 模型文件导入障碍物场。

    支持格式: .obj / .stl / .ply

    参数
    ----
    filepath : 模型文件路径
    voxel_size : 体素尺寸 (m)
    scale : 顶点坐标统一缩放因子
    padding : 模型包围盒外扩距离 (m)
    max_obstacles : 最大 AABB 障碍物数量

    返回
    ----
    (ObstacleField, bounds) 其中 bounds shape=(2,3)
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"模型文件不存在: {filepath}")

    mesh = parse_model_bytes(path.read_bytes(), filename=path.name)
    return _model_to_obstacle_field(
        mesh.vertices,
        mesh.faces,
        voxel_size=voxel_size,
        scale=scale,
        padding=padding,
        max_obstacles=max_obstacles,
    )


def import_model_to_grid(
    filepath: str,
    *,
    voxel_size: float = 0.4,
    scale: float = 1.0,
    padding: float = 0.5,
    max_obstacles: int = 500,
) -> "OccupancyGrid":
    """从 3D 模型文件导入并直接返回 OccupancyGrid。"""
    from .obstacles import OccupancyGrid

    field, bounds = import_model(
        filepath,
        voxel_size=voxel_size,
        scale=scale,
        padding=padding,
        max_obstacles=max_obstacles,
    )
    return field.to_voxel_grid(bounds, voxel_size)
