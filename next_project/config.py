"""Compatibility wrapper for simulation preset configs.

The public API stays here; preset builders live under ``configs/``.
"""

from __future__ import annotations

try:
    from configs.registry import AVAILABLE_PRESETS, get_config, load_preset_metadata
except ModuleNotFoundError:
    from .configs.registry import AVAILABLE_PRESETS, get_config, load_preset_metadata

__all__ = ["AVAILABLE_PRESETS", "get_config", "load_preset_metadata"]
