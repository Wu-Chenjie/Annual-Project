from __future__ import annotations



from core.model_importer import ImportedMesh, model_to_map_json, parse_model_bytes


def test_parse_model_bytes_obj_minimal_triangle():
    payload = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"

    mesh = parse_model_bytes(payload, filename="tri.obj")

    assert len(mesh.vertices) == 3
    assert len(mesh.faces) == 1


def test_model_to_map_json_preserves_map_payload_shape():
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
