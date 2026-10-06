"""Seconds-valued motion proxies; only the final fitted curve certifies limits."""
import math
import numpy as np


def stopped_motion_time(distance, speed=.6, acceleration=.8, jerk=2.):
    """Rest-to-rest quintic envelope, including short-leg derivative costs."""
    if distance < 0 or not math.isfinite(distance):
        return float('inf')
    return max(1.875*distance/speed, math.sqrt(5.773503*distance/acceleration),
               (60.*distance/jerk)**(1./3.))


def view_service_cost(path, yaw, target_yaw, recent=(), observation_s=1.5):
    points = np.asarray(path, float)
    legs = np.diff(points, axis=0); lengths = np.linalg.norm(legs, axis=1)
    legs = legs[lengths > 1e-8]; lengths = lengths[lengths > 1e-8]
    distance = float(lengths.sum())
    straight = distance/.6
    translation = stopped_motion_time(distance)
    turning = 0.
    if len(legs) > 1:
        directions = legs/lengths[:, None]
        turning = .6/.8*float(np.arccos(np.clip(np.sum(directions[1:]*directions[:-1], axis=1), -1., 1.)).sum())
    angle = abs(math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw)))
    rotation = 1.875*angle/.65
    # Translation and yaw run together. Revisited travel is a soft cost; a
    # one-cell view remains eligible, and necessary transit is never removed.
    repeated = 0.
    if recent and len(points) > 1:
        samples = []; weights = []
        for a, b in zip(points[:-1], points[1:]):
            d = float(np.linalg.norm(b-a)); n = max(1, math.ceil(d/.3))
            samples.extend(a+(b-a)*(k+.5)/n for k in range(n)); weights.extend([d/n]*n)
        old = np.asarray([p for p, _ in recent], float)
        near = np.min(np.linalg.norm(np.asarray(samples)[:, None]-old[None], axis=2), axis=1) < .65
        repeated = float(np.asarray(weights)[near].sum())/.6*.25
    motion = max(translation+turning, rotation)
    return dict(cruise_s=straight, acceleration_deceleration_s=translation-straight,
                path_turn_s=turning, yaw_s=rotation, motion_s=motion,
                observation_s=observation_s, repeated_travel_s=repeated,
                total_s=motion+observation_s+repeated)
