from __future__ import annotations

import asyncio
from io import BytesIO

import pytest


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
