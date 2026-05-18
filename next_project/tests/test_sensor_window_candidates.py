import numpy as np

from core.obstacles import ObstacleField
from core.sensors import RangeSensor6


def _build_dense_field() -> ObstacleField:
    field = ObstacleField()
    for ix in range(-12, 13):
        for iy in range(-12, 13):
            # Most obstacles are outside the six sensor rays but inside the same world.
            field.add_aabb([ix * 0.8, iy * 0.8 + 0.25, -0.15],
                           [ix * 0.8 + 0.25, iy * 0.8 + 0.45, 0.15])
    field.add_aabb([2.0, -0.1, -0.1], [2.4, 0.1, 0.1])
    field.add_sphere([-3.0, 0.0, 0.0], 0.25)
    field.add_cylinder([0.0, 3.0], 0.2, -0.4, 0.4)
    return field


def _exhaustive_ray_cast(sensor: RangeSensor6, origin: np.ndarray, direction: np.ndarray, field: ObstacleField) -> float:
    best = sensor.max_range
    for obs in field._obstacles:
        t = sensor._ray_obs_intersect(origin, direction, obs)
        if 0.0 < t < best:
            best = t
    return best


def test_window_candidates_match_exhaustive_sensor_distances():
    field = _build_dense_field()
    sensor = RangeSensor6(max_range=5.0, noise_std=0.0, seed=123)
    pose = np.array([0.0, 0.0, 0.0])

    accelerated = sensor.sense(pose, field)
    exhaustive = np.array([
        _exhaustive_ray_cast(sensor, pose, direction, field)
        for direction in sensor.DIRECTIONS
    ])

    assert np.allclose(accelerated, exhaustive, atol=1e-12)


def test_window_candidates_exclude_obstacles_outside_sensor_range_window():
    field = _build_dense_field()
    origin = np.array([0.0, 0.0, 0.0])
    direction = np.array([1.0, 0.0, 0.0])

    candidates = field.obstacles_in_ray_window(origin, direction, max_range=5.0)

    assert len(candidates) < len(field._obstacles) // 4
    assert any(np.isclose(RangeSensor6._ray_obs_intersect(origin, direction, obs), 2.0) for obs in candidates)
