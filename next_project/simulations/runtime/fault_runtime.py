from __future__ import annotations

from core.fault_detector import FaultDetector


class FaultRuntime:
    def _build_fault_detector(self) -> FaultDetector | None:
        cfg = self.config
        if not getattr(cfg, "fault_detection_enabled", False):
            return None
        return FaultDetector(
            max_acc=cfg.fault_detector_max_acc,
            pos_dev_threshold=cfg.fault_detector_pos_dev,
            saturate_steps=cfg.fault_detector_saturate_steps,
            dt=self.dt,
        )
