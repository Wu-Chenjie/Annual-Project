"""Dynamic replay verification server.

Run from this directory:
    uvicorn server:app --host 0.0.0.0 --port 8765
"""

from __future__ import annotations

import json
import os
import re
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile
from fastapi.responses import HTMLResponse, Response


app = FastAPI(title="Dynamic Replay Server")

PROJECT_ROOT = Path(__file__).resolve().parent.parent
import sys
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
try:
    from core.result_schema import build_web_sim_result_payload
except ImportError:  # pragma: no cover
    build_web_sim_result_payload = None  # type: ignore[assignment]
MAPS_DIR = PROJECT_ROOT / "maps"
WEB_DIR = PROJECT_ROOT / "web"
RECONSTRUCTION_DIR = PROJECT_ROOT / "artifacts" / "reconstruction"
COLMAP_BIN = Path(r"D:\tools\photogrammetry\colmap-4.0.4\bin\colmap.exe")
OPENMVS_DIR = Path(r"D:\tools\photogrammetry\openmvs-2.4.0\vc17\x64\Release")
RECON_JOBS: dict[str, dict[str, Any]] = {}
RECON_LOCK = threading.Lock()


def _candidate_executables() -> list[Path]:
    name = "sim_dynamic_replay.exe" if os.name == "nt" else "sim_dynamic_replay"
    return [
        PROJECT_ROOT / "cpp" / "build" / "Release" / name,
        PROJECT_ROOT / "cpp" / "build" / "Debug" / name,
        PROJECT_ROOT / "cpp" / "build" / name,
        PROJECT_ROOT / "cpp" / "build_subst" / name,
    ]


def _resolve_executable() -> Path:
    for path in _candidate_executables():
        if path.exists():
            return path
    tried = "\n".join(str(p) for p in _candidate_executables())
    raise HTTPException(
        500,
        "sim_dynamic_replay executable not found. Build it first with "
        "`cmake --build cpp/build --config Release`, or on Windows run "
        "`powershell -ExecutionPolicy Bypass -File cpp/build_windows.ps1`.\nTried:\n" + tried,
    )


PRESETS: dict[str, str] = {
    "basic": "基础编队验证",
    "obstacle": "简单障碍物避障",
    "warehouse": "工业仓库在线A*",
    "warehouse_a": "仓库A*+ESDF+Danger",
    "warehouse_online": "仓库在线简化版",
    "warehouse_danger": "仓库GNN双模式",
    "fault_tolerance": "容错测试",
    "fault_tolerance_online": "容错在线测试",
    "school_corridor": "学校走廊离线",
    "school_corridor_online": "学校走廊在线",
    "company_cubicles": "公司格子间离线",
    "company_cubicles_online": "公司格子间在线",
    "meeting_room": "会议室离线",
    "meeting_room_online": "会议室在线",
    "laboratory": "实验室离线",
    "laboratory_online": "实验室在线",
    "custom": "自定义",
}


def _safe_map_path(name: str) -> Path:
    if "/" in name or "\\" in name or ".." in name:
        raise HTTPException(400, "Invalid map name")
    path = (MAPS_DIR / f"{name}.json").resolve()
    maps_root = MAPS_DIR.resolve()
    if maps_root not in path.parents and path != maps_root:
        raise HTTPException(400, "Invalid map path")
    return path


def _safe_name(name: str, default: str, label: str) -> str:
    stem = Path(name or default).stem
    stem = re.sub(r"[^A-Za-z0-9_-]+", "_", stem).strip("_")
    if not stem:
        raise HTTPException(400, f"{label} must contain letters, numbers, _ or -")
    return stem[:64]


def _safe_new_map_name(name: str) -> str:
    return _safe_name(name, "imported_model", "Map name")


def _safe_scene_name(name: str) -> str:
    return _safe_name(name, "photo_scene", "Scene name")


def _safe_upload_filename(filename: str, index: int) -> str:
    raw = Path(filename or f"image_{index:03d}.jpg").name
    suffix = Path(raw).suffix.lower()
    if suffix not in {".jpg", ".jpeg", ".png", ".webp", ".bmp", ".tif", ".tiff"}:
        raise HTTPException(400, f"Unsupported image format: {suffix or '(none)'}")
    stem = re.sub(r"[^A-Za-z0-9_-]+", "_", Path(raw).stem).strip("_") or f"image_{index:03d}"
    return f"{index:03d}_{stem[:48]}{suffix}"


def _require_reconstruction_image_count(count: int) -> None:
    if count < 2:
        raise HTTPException(400, "Reconstruction needs at least 2 photos; 10+ overlapping indoor photos is more realistic.")


def _reconstruction_commands(scene: str) -> dict[str, list[str]]:
    root = str((RECONSTRUCTION_DIR / scene).resolve())
    colmap = str(COLMAP_BIN)
    interface_colmap = str((OPENMVS_DIR / "InterfaceCOLMAP.exe").resolve())
    densify = str((OPENMVS_DIR / "DensifyPointCloud.exe").resolve())
    reconstruct = str((OPENMVS_DIR / "ReconstructMesh.exe").resolve())
    refine = str((OPENMVS_DIR / "RefineMesh.exe").resolve())
    texture = str((OPENMVS_DIR / "TextureMesh.exe").resolve())
    return {
        "colmap_openmvs": [
            f'New-Item -ItemType Directory -Force -Path "{root}\\colmap", "{root}\\openmvs" | Out-Null',
            f'& "{colmap}" automatic_reconstructor --workspace_path "{root}\\colmap" --image_path "{root}\\images"',
            f'& "{interface_colmap}" -i "{root}\\colmap\\dense\\0" -o "{root}\\openmvs\\scene.mvs" --image-folder "{root}\\colmap\\dense\\0\\images"',
            f'& "{densify}" "{root}\\openmvs\\scene.mvs" -w "{root}\\openmvs"',
            f'& "{reconstruct}" "{root}\\openmvs\\scene_dense.mvs" -w "{root}\\openmvs"',
            f'& "{refine}" "{root}\\openmvs\\scene_dense_mesh.mvs" -w "{root}\\openmvs"',
            f'& "{texture}" "{root}\\openmvs\\scene_dense_mesh_refine.mvs" -w "{root}\\openmvs"',
        ],
    }


def _job_snapshot(job: dict[str, Any]) -> dict[str, Any]:
    return {
        "job_id": job["job_id"],
        "scene": job["scene"],
        "status": job["status"],
        "step": job.get("step"),
        "message": job.get("message", ""),
        "image_dir": job.get("image_dir"),
        "map_name": job.get("map_name"),
        "map_path": job.get("map_path"),
        "model_path": job.get("model_path"),
        "error": job.get("error"),
        "updated_at": job.get("updated_at"),
    }


def _set_job_state(job_id: str, **updates: Any) -> None:
    with RECON_LOCK:
        job = RECON_JOBS.get(job_id)
        if not job:
            return
        job.update(updates)
        job["updated_at"] = time.time()


def _run_checked(command: list[str], cwd: Path, job_id: str, step: str) -> subprocess.CompletedProcess[str]:
    _set_job_state(job_id, status="running", step=step, message=" ".join(command))
    result = subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"{step} failed with exit code {result.returncode}\n"
            f"stdout:\n{result.stdout[-4000:]}\n"
            f"stderr:\n{result.stderr[-4000:]}"
        )
    return result


def _pick_model_file(openmvs_dir: Path) -> Path:
    candidates = [
        openmvs_dir / "scene_dense_mesh_refine_texture.ply",
        openmvs_dir / "scene_dense_mesh_refine.ply",
        openmvs_dir / "scene_dense_mesh_texture.ply",
        openmvs_dir / "scene_dense_mesh.ply",
        openmvs_dir / "scene_dense_mesh_refine.obj",
        openmvs_dir / "scene_dense_mesh.obj",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"No reconstructed mesh found in {openmvs_dir}")


def _run_reconstruction_job(job_id: str) -> None:
    with RECON_LOCK:
        job = dict(RECON_JOBS[job_id])
    scene = job["scene"]
    scene_root = RECONSTRUCTION_DIR / scene
    colmap_root = scene_root / "colmap"
    openmvs_root = scene_root / "openmvs"
    map_name = scene
    try:
        image_files = [p for p in (scene_root / "images").iterdir() if p.is_file()]
        if len(image_files) < 2:
            raise RuntimeError("Reconstruction needs at least 2 photos. Current scene has fewer than 2 uploaded images.")
        colmap_root.mkdir(parents=True, exist_ok=True)
        openmvs_root.mkdir(parents=True, exist_ok=True)

        _run_checked(
            [
                str(COLMAP_BIN),
                "automatic_reconstructor",
                "--workspace_path",
                str(colmap_root),
                "--image_path",
                str(scene_root / "images"),
            ],
            PROJECT_ROOT,
            job_id,
            "colmap_automatic_reconstructor",
        )
        sparse_root = colmap_root / "dense" / "0" / "sparse"
        if not (sparse_root / "cameras.bin").exists() and not (sparse_root / "cameras.txt").exists():
            raise RuntimeError(
                "COLMAP did not produce dense/0/sparse cameras output. "
                "This usually means too few photos, not enough overlap, or failed feature matching."
            )
        _run_checked(
            [
                str(OPENMVS_DIR / "InterfaceCOLMAP.exe"),
                "-i",
                str(colmap_root / "dense" / "0"),
                "-o",
                str(openmvs_root / "scene.mvs"),
                "--image-folder",
                str(colmap_root / "dense" / "0" / "images"),
            ],
            PROJECT_ROOT,
            job_id,
            "openmvs_interface_colmap",
        )
        _run_checked(
            [str(OPENMVS_DIR / "DensifyPointCloud.exe"), str(openmvs_root / "scene.mvs"), "-w", str(openmvs_root)],
            PROJECT_ROOT,
            job_id,
            "openmvs_densify_point_cloud",
        )
        _run_checked(
            [str(OPENMVS_DIR / "ReconstructMesh.exe"), str(openmvs_root / "scene_dense.mvs"), "-w", str(openmvs_root)],
            PROJECT_ROOT,
            job_id,
            "openmvs_reconstruct_mesh",
        )

        model_path = _pick_model_file(openmvs_root)
        _set_job_state(job_id, status="running", step="import_model_map", model_path=str(model_path), message="Importing mesh into maps")
        vertices, triangles = _parse_model(model_path.name, model_path.read_bytes())
        map_json = _model_to_map(
            vertices,
            triangles,
            voxel_size=0.5,
            scale=1.0,
            padding=1.0,
            max_obstacles=4000,
            source_name=model_path.name,
        )
        map_path = _safe_map_path(map_name)
        map_path.write_text(json.dumps(map_json, indent=2, ensure_ascii=False), encoding="utf-8")
        _set_job_state(
            job_id,
            status="completed",
            step="completed",
            message=f"Generated map {map_name}",
            map_name=map_name,
            map_path=str(map_path),
            model_path=str(model_path),
        )
    except Exception as exc:
        _set_job_state(job_id, status="failed", step="failed", error=str(exc), message="Reconstruction failed")


def _vec_cell(point: tuple[float, float, float], voxel: float) -> tuple[int, int, int]:
    return (
        int(point[0] // voxel),
        int(point[1] // voxel),
        int(point[2] // voxel),
    )


def _parse_obj(text: str) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
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


def _parse_stl(data: bytes) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
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


def _parse_ply(data: bytes) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    marker = b"end_header"
    header_end = data.find(marker)
    if header_end < 0:
        raise HTTPException(400, "PLY header missing end_header")
    header_stop = data.find(b"\n", header_end)
    if header_stop < 0:
        header_stop = header_end + len(marker)
    header = data[:header_stop].decode("utf-8", errors="ignore")
    lines = header.splitlines()
    if not lines or lines[0].strip() != "ply":
        raise HTTPException(400, "Invalid PLY file")
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
        raise HTTPException(400, f"Unsupported PLY format: {fmt}")
    return _parse_binary_little_ply(data[header_stop + 1:], vertex_count, face_count, vertex_properties)


def _parse_ascii_ply(
    body: str,
    vertex_count: int,
    face_count: int,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    lines = body.splitlines()
    vertices: list[tuple[float, float, float]] = []
    for raw in lines[:vertex_count]:
        parts = raw.split()
        if len(parts) >= 3:
            vertices.append((float(parts[0]), float(parts[1]), float(parts[2])))
    triangles: list[tuple[int, int, int]] = []
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
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    if not vertex_properties:
        vertex_properties = [("x", "float"), ("y", "float"), ("z", "float")]
    offset = 0
    vertices: list[tuple[float, float, float]] = []
    for _ in range(vertex_count):
        values: dict[str, float] = {}
        for name, typ in vertex_properties:
            info = PLY_SCALARS.get(typ)
            if not info:
                raise HTTPException(400, f"Unsupported PLY vertex property type: {typ}")
            code, size = info
            if offset + size > len(body):
                raise HTTPException(400, "Unexpected end of binary PLY vertex data")
            values[name] = float(struct.unpack_from("<" + code, body, offset)[0])
            offset += size
        vertices.append((values.get("x", 0.0), values.get("y", 0.0), values.get("z", 0.0)))

    triangles: list[tuple[int, int, int]] = []
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


def _parse_model(filename: str, data: bytes) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    ext = Path(filename).suffix.lower()
    if ext == ".obj":
        return _parse_obj(data.decode("utf-8", errors="ignore"))
    if ext == ".stl":
        return _parse_stl(data)
    if ext == ".ply":
        return _parse_ply(data)
    raise HTTPException(400, "Supported model formats: .stl, .obj, .ply")


def _sample_triangle(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
    voxel: float,
) -> list[tuple[float, float, float]]:
    edge = max(
        ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5,
        ((a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2 + (a[2] - c[2]) ** 2) ** 0.5,
        ((b[0] - c[0]) ** 2 + (b[1] - c[1]) ** 2 + (b[2] - c[2]) ** 2) ** 0.5,
    )
    steps = max(1, min(48, int(edge / max(voxel * 0.5, 1e-6)) + 1))
    out: list[tuple[float, float, float]] = []
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


def _model_to_map(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    *,
    voxel_size: float,
    scale: float,
    padding: float,
    max_obstacles: int,
    source_name: str,
) -> dict[str, Any]:
    if not vertices:
        raise HTTPException(400, "Model contains no vertices")
    if voxel_size <= 0 or scale <= 0:
        raise HTTPException(400, "voxel_size and scale must be positive")

    scaled = [(x * scale, y * scale, z * scale) for x, y, z in vertices]
    min_x = min(p[0] for p in scaled)
    min_y = min(p[1] for p in scaled)
    min_z = min(p[2] for p in scaled)
    shifted = [(x - min_x + padding, y - min_y + padding, z - min_z) for x, y, z in scaled]

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
        raise HTTPException(
            413,
            f"Converted model produced {len(runs)} obstacles. Increase voxel size or max_obstacles.",
        )

    obstacles = [
        {
            "type": "aabb",
            "min": [round(x0 * voxel_size, 4), round(iy * voxel_size, 4), round(iz * voxel_size, 4)],
            "max": [round((x1 + 1) * voxel_size, 4), round((iy + 1) * voxel_size, 4), round((iz + 1) * voxel_size, 4)],
        }
        for x0, x1, iy, iz in runs
    ]

    max_x = max(p[0] for p in shifted) + padding
    max_y = max(p[1] for p in shifted) + padding
    max_z = max(p[2] for p in shifted) + max(voxel_size, padding * 0.25)
    return {
        "bounds": [[0, 0, 0], [round(max_x, 4), round(max_y, 4), round(max_z, 4)]],
        "description": f"Imported from 3D model {source_name}; voxel_size={voxel_size}, scale={scale}",
        "obstacles": obstacles,
    }


@app.get("/api/presets")
async def get_presets() -> dict[str, Any]:
    return {"presets": [{"id": key, "name": name} for key, name in PRESETS.items()]}


@app.get("/api/maps")
async def list_maps() -> dict[str, Any]:
    return {"maps": sorted(p.stem for p in MAPS_DIR.glob("*.json"))}


@app.get("/api/maps/{name}")
async def get_map(name: str) -> Any:
    path = _safe_map_path(name)
    if not path.exists():
        raise HTTPException(404, f"Map not found: {name}")
    return json.loads(path.read_text(encoding="utf-8"))


@app.get("/assets/plotly.min.js")
async def plotly_js() -> Response:
    try:
        from plotly.offline.offline import get_plotlyjs
    except Exception as exc:
        raise HTTPException(500, f"Plotly is not installed in this Python environment: {exc}") from exc
    return Response(get_plotlyjs(), media_type="application/javascript")


@app.post("/api/maps/validate")
async def validate_map(request: Request) -> dict[str, Any]:
    body = await request.json()
    if not isinstance(body.get("bounds"), list) or not isinstance(body.get("obstacles"), list):
        raise HTTPException(400, "Map JSON must contain bounds and obstacles arrays")
    return {"ok": True, "obstacle_count": len(body["obstacles"])}


@app.post("/api/maps/save")
async def save_map(request: Request) -> dict[str, Any]:
    body = await request.json()
    name = _safe_new_map_name(str(body.get("name") or "custom_map"))
    map_json = body.get("map")
    if not isinstance(map_json, dict):
        raise HTTPException(400, "Request must contain a map object")
    if not isinstance(map_json.get("bounds"), list) or not isinstance(map_json.get("obstacles"), list):
        raise HTTPException(400, "Map JSON must contain bounds and obstacles arrays")
    if "waypoints" in map_json and not isinstance(map_json["waypoints"], list):
        raise HTTPException(400, "waypoints must be an array")
    path = _safe_map_path(name)
    path.write_text(json.dumps(map_json, indent=2, ensure_ascii=False), encoding="utf-8")
    return {
        "ok": True,
        "map": name,
        "path": str(path),
        "obstacle_count": len(map_json["obstacles"]),
        "waypoint_count": len(map_json.get("waypoints") or []),
    }


@app.post("/api/maps/import-model")
async def import_model_map(
    file: UploadFile = File(...),
    map_name: str = Form("imported_model"),
    voxel_size: float = Form(0.5),
    scale: float = Form(1.0),
    padding: float = Form(1.0),
    max_obstacles: int = Form(4000),
) -> dict[str, Any]:
    data = await file.read()
    if not data:
        raise HTTPException(400, "Empty model file")
    name = _safe_new_map_name(map_name)
    vertices, triangles = _parse_model(file.filename or name, data)
    map_json = _model_to_map(
        vertices,
        triangles,
        voxel_size=voxel_size,
        scale=scale,
        padding=max(0.0, padding),
        max_obstacles=max(1, max_obstacles),
        source_name=file.filename or name,
    )
    path = _safe_map_path(name)
    path.write_text(json.dumps(map_json, indent=2, ensure_ascii=False), encoding="utf-8")
    return {
        "ok": True,
        "map": name,
        "path": str(path),
        "vertex_count": len(vertices),
        "triangle_count": len(triangles),
        "obstacle_count": len(map_json["obstacles"]),
        "bounds": map_json["bounds"],
    }


@app.post("/api/reconstruction/photo-set")
async def upload_reconstruction_photo_set(
    files: list[UploadFile] = File(...),
    scene_name: str = Form("photo_scene"),
) -> dict[str, Any]:
    scene = _safe_scene_name(scene_name)
    if not files:
        raise HTTPException(400, "Upload at least one image")
    if len(files) > 250:
        raise HTTPException(413, "Too many images; upload at most 250 at once")
    _require_reconstruction_image_count(len(files))

    image_dir = RECONSTRUCTION_DIR / scene / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    saved: list[str] = []
    total_bytes = 0
    for index, upload in enumerate(files, start=1):
        filename = _safe_upload_filename(upload.filename or "", index)
        data = await upload.read()
        if not data:
            raise HTTPException(400, f"Empty image file: {upload.filename or filename}")
        total_bytes += len(data)
        if total_bytes > 2_000_000_000:
            raise HTTPException(413, "Uploaded image set is too large")
        path = image_dir / filename
        path.write_bytes(data)
        saved.append(filename)

    commands = _reconstruction_commands(scene)
    return {
        "ok": True,
        "scene": scene,
        "image_count": len(saved),
        "image_dir": str(image_dir),
        "saved_files": saved,
        "commands": commands,
        "next_step": "Run one reconstruction command chain, export .ply/.obj/.stl, then import it with /api/maps/import-model.",
    }


@app.post("/api/reconstruction/run")
async def run_reconstruction(
    files: list[UploadFile] = File(...),
    scene_name: str = Form("photo_scene"),
) -> dict[str, Any]:
    upload_result = await upload_reconstruction_photo_set(files=files, scene_name=scene_name)
    job_id = uuid.uuid4().hex[:12]
    job = {
        "job_id": job_id,
        "scene": upload_result["scene"],
        "status": "queued",
        "step": "queued",
        "message": "Queued reconstruction job",
        "image_dir": upload_result["image_dir"],
        "map_name": upload_result["scene"],
        "updated_at": time.time(),
    }
    with RECON_LOCK:
        RECON_JOBS[job_id] = job
    thread = threading.Thread(target=_run_reconstruction_job, args=(job_id,), daemon=True)
    thread.start()
    return {"ok": True, "job": _job_snapshot(job)}


@app.get("/api/reconstruction/jobs/{job_id}")
async def get_reconstruction_job(job_id: str) -> dict[str, Any]:
    with RECON_LOCK:
        job = RECON_JOBS.get(job_id)
    if not job:
        raise HTTPException(404, f"Reconstruction job not found: {job_id}")
    return {"ok": True, "job": _job_snapshot(job)}


@app.post("/api/simulate")
async def simulate(request: Request) -> dict[str, Any]:
    body = await request.json()
    exe = _resolve_executable()
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        input_path = tmp / "input.json"
        output_path = tmp / "output.json"
        sim_input = dict(body)
        map_name = sim_input.get("map_file") or sim_input.get("base_config", {}).get("map_file")
        if isinstance(map_name, str) and map_name:
            if "/" not in map_name and "\\" not in map_name and ".." not in map_name:
                source_map = _safe_map_path(Path(map_name).stem)
                if source_map.exists():
                    tmp_map = tmp / source_map.name
                    shutil.copyfile(source_map, tmp_map)
                    sim_input["map_file"] = str(tmp_map)
                    base_config = dict(sim_input.get("base_config") or {})
                    try:
                        source_map_json = json.loads(source_map.read_text(encoding="utf-8"))
                    except Exception:
                        source_map_json = {}
                    if "waypoints" not in base_config and isinstance(source_map_json.get("waypoints"), list):
                        base_config["waypoints"] = source_map_json["waypoints"]
                    base_config["map_file"] = str(tmp_map)
                    sim_input["base_config"] = base_config
        input_path.write_text(json.dumps(sim_input, indent=2, ensure_ascii=False), encoding="utf-8")
        ts_start = time.perf_counter()
        result = subprocess.run(
            [str(exe), str(input_path), "-o", str(output_path)],
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=300,
        )
        ts_elapsed = time.perf_counter() - ts_start
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or f"unknown C++ error (exit {result.returncode})"
            raise HTTPException(500, f"C++ simulation failed: {detail}")
        if not output_path.exists():
            raise HTTPException(500, "C++ simulation did not produce output.json")
        raw = json.loads(output_path.read_text(encoding="utf-8"))
        preset = body.get("preset") or body.get("base_config", {}).get("preset", "custom")
        if build_web_sim_result_payload is not None:
            return build_web_sim_result_payload(preset=preset, web_results=raw, runtime_s=ts_elapsed)
        return {"results": raw}


@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse(
        (WEB_DIR / "dynamic_replay.html").read_text(encoding="utf-8"),
        headers={"Cache-Control": "no-store, max-age=0"},
    )
