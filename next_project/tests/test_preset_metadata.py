from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import AVAILABLE_PRESETS, load_preset_metadata
from web.server import PRESETS


def test_metadata_covers_available_presets() -> None:
    metadata = load_preset_metadata()

    missing = set(AVAILABLE_PRESETS) - set(metadata)
    extra = set(metadata) - set(AVAILABLE_PRESETS)

    assert not missing
    assert not extra


def test_preset_metadata_entries_have_required_fields() -> None:
    metadata = load_preset_metadata()

    for preset in AVAILABLE_PRESETS:
        entry = metadata[preset]
        assert entry["label"]
        assert entry["mode"] in {"offline", "online", "custom"}
        assert entry["description"]


def test_web_preset_list_covers_config_presets() -> None:
    missing = set(AVAILABLE_PRESETS) - set(PRESETS)
    assert not missing


def test_web_preset_labels_come_from_metadata() -> None:
    metadata = load_preset_metadata()

    assert PRESETS == {
        preset: metadata[preset]["label"]
        for preset in AVAILABLE_PRESETS
    }
