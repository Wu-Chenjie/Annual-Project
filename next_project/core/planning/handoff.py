"""Exact curve boundary checks; no controller or reservation side effects."""
import math
import numpy as np


def future_boundary_time(curve, progress, lead):
    """Earliest future boundary with enough planning/authorization lead.

    Actual old-service gain is checked separately before adoption. Choosing a
    boundary does not authorize leaving, release a lease, or complete a task.
    """
    if not np.isfinite([progress,lead]).all() or progress<0 or lead<=0:
        raise ValueError('Invalid handoff progress or lead')
    boundary=progress+lead
    return float(boundary) if boundary<curve.duration-.4 else None


def moving_boundary_time(curve, progress, lead):
    """Latest feasible boundary before longitudinal deceleration, never clip lead.

    Preparing near the end of cruise gives actual old-service frames time to
    arrive. When preparation is already late, preserve the existing feasible
    boundary during deceleration. Never manufacture a window by clipping lead.
    """
    earliest = future_boundary_time(curve, progress, lead)
    if earliest is None:
        return None
    times = np.linspace(earliest, curve.duration-.4, max(2, int(curve.duration/.05)+1))
    _, velocities, accelerations, _, _ = curve.sample_many(times)
    speeds = np.linalg.norm(velocities, axis=1)
    increasing = np.sum(velocities*accelerations, axis=1) >= -1e-4
    candidates = times[(speeds >= .1) & increasing]
    return float(candidates[-1]) if len(candidates) else earliest


def continuity(old, new, progress):
    left = old.sample(progress); right = new.sample(0.)
    return dict(position=float(np.linalg.norm(left[0]-right[0])),
                velocity=float(np.linalg.norm(left[1]-right[1])),
                acceleration=float(np.linalg.norm(left[2]-right[2])),
                yaw=abs(math.atan2(math.sin(left[3]-right[3]), math.cos(left[3]-right[3]))),
                yaw_rate=abs(left[4]-right[4]),
                yaw_acceleration=abs(old.yaw_acceleration(progress)-new.yaw_acceleration(0.)))


def validate_handoff(old, new, progress, current_progress=0.):
    if not np.isfinite(progress) or not current_progress+.05 < progress < old.duration:
        raise ValueError('Handoff is outside the unconsumed curve')
    errors = continuity(old, new, progress)
    if any(value > 1e-5 for value in errors.values()):
        raise ValueError('Handoff violates position / C2 / yaw boundary continuity')
    return errors
