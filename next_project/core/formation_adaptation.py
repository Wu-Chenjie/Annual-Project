"""Adaptive formation selection for constrained indoor flight."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence


Envelope = tuple[float, float, float]
ChannelWidth = tuple[float, float, float] | None


@dataclass(frozen=True)
class FormationAdaptationDecision:
    should_switch: bool
    target_formation: str
    reason: str
    current_formation: str
    channel_width: ChannelWidth
    selected_envelope: Envelope | None
    clearance_margin: float | None
    target_spacing: float | None = None
    selected_min_pair_distance: float | None = None
    blocked_by_hold_time: bool = False

    def to_event(self, time_now: float) -> dict[str, object]:
        return {
            "t": float(time_now),
            "from": self.current_formation,
            "to": self.target_formation,
            "reason": self.reason,
            "channel_width": None if self.channel_width is None else list(self.channel_width),
            "selected_envelope": None if self.selected_envelope is None else list(self.selected_envelope),
            "clearance_margin": self.clearance_margin,
            "target_spacing": self.target_spacing,
            "selected_min_pair_distance": self.selected_min_pair_distance,
            "blocked_by_hold_time": bool(self.blocked_by_hold_time),
        }


class FormationAdaptationPolicy:
    """Choose a formation that fits channel geometry and clearance feedback."""

    def __init__(
        self,
        *,
        formations: Sequence[str] = ("diamond", "v_shape", "triangle", "line"),
        default_formation: str = "diamond",
        fit_scale: float = 2.0,
        min_hold_time_s: float = 1.0,
        recovery_clearance_margin: float = 0.15,
    ) -> None:
        if not formations:
            raise ValueError("formations must not be empty")
        self.formations = tuple(dict.fromkeys(str(item) for item in formations))
        self.default_formation = str(default_formation)
        self.fit_scale = max(float(fit_scale), 1e-6)
        self.min_hold_time_s = max(float(min_hold_time_s), 0.0)
        self.recovery_clearance_margin = float(recovery_clearance_margin)

    def decide(
        self,
        *,
        time_now: float,
        current_formation: str,
        channel_width: ChannelWidth,
        envelope_by_formation: Mapping[str, Envelope],
        spacing_by_formation: Mapping[str, float] | None = None,
        min_pair_distance_by_formation: Mapping[str, float] | None = None,
        min_inter_drone_distance: float | None = None,
        clearance_margin: float | None,
        last_switch_time_s: float | None,
    ) -> FormationAdaptationDecision:
        current = str(current_formation)
        width = self._normalize_channel(channel_width)
        margin = None if clearance_margin is None else float(clearance_margin)
        current_env = self._envelope_for(current, envelope_by_formation)
        current_fits = self._fits(width, current_env)
        hold_blocked = self._hold_blocks(float(time_now), last_switch_time_s)
        clearance_bad = margin is not None and margin < 0.0

        if clearance_bad or not current_fits:
            target = self._best_safe_candidate(
                width,
                envelope_by_formation,
                min_pair_distance_by_formation=min_pair_distance_by_formation,
                min_inter_drone_distance=min_inter_drone_distance,
            )
            reason = "clearance_violation" if clearance_bad else "channel_too_narrow"
            return FormationAdaptationDecision(
                should_switch=target != current,
                target_formation=target,
                reason=reason,
                current_formation=current,
                channel_width=width,
                selected_envelope=self._envelope_for(target, envelope_by_formation),
                clearance_margin=margin,
                target_spacing=self._spacing_for(target, spacing_by_formation),
                selected_min_pair_distance=self._min_pair_for(target, min_pair_distance_by_formation),
                blocked_by_hold_time=False,
            )

        default_env = self._envelope_for(self.default_formation, envelope_by_formation)
        can_recover = (
            current != self.default_formation
            and not hold_blocked
            and self._fits(width, default_env)
            and margin is not None
            and margin >= self.recovery_clearance_margin
        )
        if can_recover:
            return FormationAdaptationDecision(
                should_switch=True,
                target_formation=self.default_formation,
                reason="recover_default",
                current_formation=current,
                channel_width=width,
                selected_envelope=default_env,
                clearance_margin=margin,
                target_spacing=self._spacing_for(self.default_formation, spacing_by_formation),
                selected_min_pair_distance=self._min_pair_for(
                    self.default_formation,
                    min_pair_distance_by_formation,
                ),
                blocked_by_hold_time=False,
            )

        return FormationAdaptationDecision(
            should_switch=False,
            target_formation=current,
            reason="hold_current" if hold_blocked else "current_safe",
            current_formation=current,
            channel_width=width,
            selected_envelope=current_env,
            clearance_margin=margin,
            target_spacing=self._spacing_for(current, spacing_by_formation),
            selected_min_pair_distance=self._min_pair_for(current, min_pair_distance_by_formation),
            blocked_by_hold_time=hold_blocked,
        )

    def _best_safe_candidate(
        self,
        channel_width: ChannelWidth,
        envelope_by_formation: Mapping[str, Envelope],
        *,
        min_pair_distance_by_formation: Mapping[str, float] | None = None,
        min_inter_drone_distance: float | None = None,
    ) -> str:
        candidates = [name for name in self.formations if name in envelope_by_formation]
        if not candidates:
            return self.default_formation
        pair_safe = [
            name for name in candidates
            if self._pair_distance_safe(name, min_pair_distance_by_formation, min_inter_drone_distance)
        ]
        pair_pool = pair_safe if pair_safe else candidates
        fitting = [
            name for name in pair_pool
            if self._fits(channel_width, self._envelope_for(name, envelope_by_formation))
        ]
        pool = fitting if fitting else pair_pool
        if channel_width is not None and not fitting:
            return min(
                pool,
                key=lambda name: self._channel_deficit_score(
                    channel_width,
                    self._envelope_for(name, envelope_by_formation),
                ),
            )
        return min(pool, key=lambda name: self._compactness_score(self._envelope_for(name, envelope_by_formation)))

    def _compactness_score(self, envelope: Envelope) -> tuple[float, float, float]:
        lateral, longitudinal, vertical = envelope
        return (float(lateral), float(max(lateral, longitudinal, vertical)), float(lateral + longitudinal + vertical))

    def _channel_deficit_score(self, channel_width: ChannelWidth, envelope: Envelope | None) -> tuple[float, float, float]:
        if channel_width is None or envelope is None:
            return (0.0, 0.0, 0.0)
        deficits = [
            max(self.fit_scale * float(axis) - float(width), 0.0)
            for width, axis in zip(channel_width, envelope)
        ]
        lateral_deficit, longitudinal_deficit, vertical_deficit = deficits
        return (
            float(sum(deficits)),
            float(lateral_deficit),
            float(max(lateral_deficit, longitudinal_deficit, vertical_deficit)),
        )

    @staticmethod
    def _spacing_for(name: str, spacing_by_formation: Mapping[str, float] | None) -> float | None:
        if spacing_by_formation is None or name not in spacing_by_formation:
            return None
        return float(spacing_by_formation[name])

    @staticmethod
    def _min_pair_for(name: str, min_pair_distance_by_formation: Mapping[str, float] | None) -> float | None:
        if min_pair_distance_by_formation is None or name not in min_pair_distance_by_formation:
            return None
        return float(min_pair_distance_by_formation[name])

    def _pair_distance_safe(
        self,
        name: str,
        min_pair_distance_by_formation: Mapping[str, float] | None,
        min_inter_drone_distance: float | None,
    ) -> bool:
        if min_inter_drone_distance is None or min_pair_distance_by_formation is None:
            return True
        value = self._min_pair_for(name, min_pair_distance_by_formation)
        if value is None:
            return True
        return value >= float(min_inter_drone_distance) - 1e-9

    def _fits(self, channel_width: ChannelWidth, envelope: Envelope | None) -> bool:
        if channel_width is None or envelope is None:
            return True
        return all(float(width) >= self.fit_scale * float(axis) for width, axis in zip(channel_width, envelope))

    def _hold_blocks(self, time_now: float, last_switch_time_s: float | None) -> bool:
        if last_switch_time_s is None:
            return False
        return (time_now - float(last_switch_time_s)) < self.min_hold_time_s

    @staticmethod
    def _normalize_channel(channel_width: ChannelWidth) -> ChannelWidth:
        if channel_width is None:
            return None
        width = tuple(float(v) for v in channel_width)
        if len(width) != 3:
            raise ValueError("channel_width must be length 3")
        return width

    @staticmethod
    def _envelope_for(name: str, envelope_by_formation: Mapping[str, Envelope]) -> Envelope | None:
        envelope = envelope_by_formation.get(name)
        if envelope is None:
            return None
        values = tuple(float(v) for v in envelope)
        if len(values) != 3:
            raise ValueError("formation envelope must be length 3")
        return values


__all__ = ["FormationAdaptationDecision", "FormationAdaptationPolicy"]
