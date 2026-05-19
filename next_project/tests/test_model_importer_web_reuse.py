from __future__ import annotations

import asyncio
from io import BytesIO

from fastapi import UploadFile

from core.model_importer import ImportedMesh, model_to_map_json, parse_model_bytes


def test_parse_model_bytes_obj_minimal_triangle():
    payload = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"

    mesh = parse_model_bytes(payload, filename="tri.obj")

    assert len(mesh.vertices) == 3
    assert len(mesh.faces) == 1


def test_model_to_map_json_preserves_web_payload_shape():
    mesh = ImportedMesh(
        vertices=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
        faces=[(0, 1, 2)],
        source_name="tri.obj",
    )

    payload = model_to_map_json(mesh, voxel_size=0.5, scale=1.0, padding=0.5, max_obstacles=100)

    assert payload["description"].startswith("Imported from 3D model tri.obj")
    assert isinstance(payload["bounds"], list)
    assert payload["obstacles"]
    assert payload["obstacles"][0]["type"] == "aabb"


def test_web_import_model_uses_core_importer_wrappers(monkeypatch, tmp_path):
    from web import server

    calls: list[str] = []
    mesh = ImportedMesh(vertices=[(0.0, 0.0, 0.0)], faces=[], source_name="tri.obj")

    def fake_parse(data: bytes, *, filename: str) -> ImportedMesh:
        calls.append(f"parse:{filename}:{len(data)}")
        return mesh

    def fake_model_to_map_json(imported: ImportedMesh, **kwargs):
        calls.append(f"map:{imported.source_name}:{kwargs['max_obstacles']}")
        return {
            "bounds": [[0, 0, 0], [1, 1, 1]],
            "description": "fake",
            "obstacles": [{"type": "aabb", "min": [0, 0, 0], "max": [1, 1, 1]}],
        }

    monkeypatch.setattr(server, "MAPS_DIR", tmp_path)
    monkeypatch.setattr(server, "parse_model_bytes", fake_parse)
    monkeypatch.setattr(server, "model_to_map_json", fake_model_to_map_json)

    upload = UploadFile(filename="tri.obj", file=BytesIO(b"obj"))

    async def run_request():
        return await server.import_model_map(
            file=upload,
            map_name="tri",
            voxel_size=0.5,
            scale=1.0,
            padding=1.0,
            max_obstacles=9,
        )

    response = asyncio.run(run_request())

    assert calls == ["parse:tri.obj:3", "map:tri.obj:9"]
    assert response["ok"] is True
    assert response["vertex_count"] == 1
    assert response["triangle_count"] == 0
    assert response["obstacle_count"] == 1
    assert (tmp_path / "tri.json").exists()
