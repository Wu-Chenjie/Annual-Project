from __future__ import annotations

import asyncio
import json
from io import BytesIO
from pathlib import Path

import pytest


class JsonRequest:
    def __init__(self, payload):
        self.payload = payload

    async def json(self):
        return self.payload


def test_web_defaults_to_loopback_host():
    from web.server import DEFAULT_HOST

    assert DEFAULT_HOST == "127.0.0.1"


def test_upload_size_limit_is_defined():
    from web.server import MAX_UPLOAD_BYTES

    assert MAX_UPLOAD_BYTES <= 50 * 1024 * 1024


def test_simulate_concurrency_limit_is_defined():
    from web.server import SIMULATE_CONCURRENCY_LIMIT

    assert SIMULATE_CONCURRENCY_LIMIT == 1


def test_import_model_rejects_oversized_upload(monkeypatch):
    from fastapi import HTTPException, UploadFile
    from web import server

    monkeypatch.setattr(server, "MAX_UPLOAD_BYTES", 8)
    upload = UploadFile(filename="tiny.obj", file=BytesIO(b"v 0 0 0\nv 1 0 0\n"))

    async def run_request() -> None:
        await server.import_model_map(file=upload)

    with pytest.raises(HTTPException) as exc:
        asyncio.run(run_request())

    assert exc.value.status_code == 413


def test_simulate_rejects_when_concurrency_slot_is_busy(monkeypatch):
    from fastapi import HTTPException
    from web import server

    monkeypatch.setattr(server, "_resolve_executable", lambda: (_ for _ in ()).throw(AssertionError("should not run")))
    acquired = server._simulate_semaphore.acquire(blocking=False)
    assert acquired
    try:
        async def run_request() -> None:
            await server.simulate(type("Request", (), {"json": lambda self: {}})())

        with pytest.raises(HTTPException) as exc:
            asyncio.run(run_request())
    finally:
        server._simulate_semaphore.release()

    assert exc.value.status_code == 429


def test_save_map_rejects_malformed_obstacle():
    from fastapi import HTTPException
    from web import server

    request = JsonRequest(
        {
            "name": "bad_map",
            "map": {
                "bounds": [[0, 0, 0], [1, 1, 1]],
                "obstacles": [{"type": "aabb", "min": [0, 0, 0]}],
            },
        }
    )

    with pytest.raises(HTTPException) as exc:
        asyncio.run(server.save_map(request))

    assert exc.value.status_code == 400
    assert not (server.MAPS_DIR / "bad_map.json").exists()


def test_save_map_rejects_non_object_json_body():
    from fastapi import HTTPException
    from web import server

    with pytest.raises(HTTPException) as exc:
        asyncio.run(server.save_map(JsonRequest([])))

    assert exc.value.status_code == 400


def test_import_model_rejects_unsupported_extension():
    from fastapi import HTTPException, UploadFile
    from web import server

    upload = UploadFile(filename="mesh.txt", file=BytesIO(b"not a mesh"))

    async def run_request() -> None:
        await server.import_model_map(file=upload, map_name="mesh")

    with pytest.raises(HTTPException) as exc:
        asyncio.run(run_request())

    assert exc.value.status_code == 400


def test_import_model_rejects_unsafe_numeric_params():
    from fastapi import HTTPException, UploadFile
    from web import server

    upload = UploadFile(filename="mesh.obj", file=BytesIO(b"v 0 0 0\n"))

    async def run_request() -> None:
        await server.import_model_map(file=upload, map_name="mesh", voxel_size=0.0)

    with pytest.raises(HTTPException) as exc:
        asyncio.run(run_request())

    assert exc.value.status_code == 400


def test_map_name_rejects_path_traversal():
    from fastapi import HTTPException
    from web import server

    with pytest.raises(HTTPException) as exc:
        server._safe_new_map_name("../outside")

    assert exc.value.status_code == 400


def test_reconstruction_run_rejects_when_feature_disabled(monkeypatch):
    from fastapi import HTTPException, UploadFile
    from web import server

    monkeypatch.setattr(server, "RECONSTRUCTION_EXPERIMENTAL_ENABLED", False)
    upload = UploadFile(filename="frame.jpg", file=BytesIO(b"jpeg"))

    async def run_request() -> None:
        await server.run_reconstruction(files=[upload], scene_name="scene")

    with pytest.raises(HTTPException) as exc:
        asyncio.run(run_request())

    assert exc.value.status_code == 403


def test_simulate_rejects_path_traversal_map_before_running(monkeypatch):
    from fastapi import HTTPException
    from web import server

    monkeypatch.setattr(server, "_resolve_executable", lambda: (_ for _ in ()).throw(AssertionError("should not run")))
    request = JsonRequest({"map_file": "../secret.json"})

    with pytest.raises(HTTPException) as exc:
        asyncio.run(server.simulate(request))

    assert exc.value.status_code == 400


def test_simulate_rejects_non_object_json_before_running(monkeypatch):
    from fastapi import HTTPException
    from web import server

    monkeypatch.setattr(server, "_resolve_executable", lambda: (_ for _ in ()).throw(AssertionError("should not run")))

    with pytest.raises(HTTPException) as exc:
        asyncio.run(server.simulate(JsonRequest([])))

    assert exc.value.status_code == 400


def test_simulate_rejects_non_object_base_config_before_running(monkeypatch):
    from fastapi import HTTPException
    from web import server

    monkeypatch.setattr(server, "_resolve_executable", lambda: (_ for _ in ()).throw(AssertionError("should not run")))

    with pytest.raises(HTTPException) as exc:
        asyncio.run(server.simulate(JsonRequest({"base_config": []})))

    assert exc.value.status_code == 400


def test_simulate_copies_only_named_map_from_maps_dir(monkeypatch, tmp_path):
    from web import server

    captured: dict[str, object] = {}
    source_map = tmp_path / "safe_map.json"
    source_map.write_text(json.dumps({"bounds": [[0, 0, 0], [1, 1, 1]], "obstacles": [], "waypoints": [[0, 0, 1]]}), encoding="utf-8")
    exe = tmp_path / "sim_dynamic_replay.exe"
    exe.write_text("", encoding="utf-8")

    monkeypatch.setattr(server, "_safe_map_path", lambda name: source_map if name == "safe_map" else (_ for _ in ()).throw(AssertionError(name)))
    monkeypatch.setattr(server, "_resolve_executable", lambda: exe)
    monkeypatch.setattr(server.subprocess, "run", lambda args, cwd, capture_output, text, timeout: _fake_sim_run(args, captured))

    request = JsonRequest({"map_file": "safe_map.json"})

    response = asyncio.run(server.simulate(request))

    assert response["preset"] == "custom"
    sim_input = captured["input"]
    copied_map = sim_input["map_file"]
    assert copied_map.endswith("safe_map.json")
    assert Path(copied_map).parent != server.MAPS_DIR
    assert sim_input["base_config"]["waypoints"] == [[0, 0, 1]]


def _fake_sim_run(args, captured):
    input_path = Path(args[1])
    output_path = Path(args[3])
    captured["input"] = json.loads(input_path.read_text(encoding="utf-8"))
    output_path.write_text(json.dumps({"summary": {"collision_count": 0}}), encoding="utf-8")

    return type("Completed", (), {"returncode": 0, "stderr": "", "stdout": ""})()
