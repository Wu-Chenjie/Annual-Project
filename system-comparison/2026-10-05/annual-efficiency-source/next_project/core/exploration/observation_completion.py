"""A processed native sensor frame proves an endpoint observation, never a forecast."""
from collections import deque
import hashlib
import json
import math
import numpy as np


class ObservationCompletion:
    def __init__(self, source, max_age_s=1.5):
        self.source = source; self.max_age_s = max_age_s
        self.frames = deque(maxlen=16); self.session = None; self.sequence = -1
        self.retired_sessions = set(); self.last_time = -math.inf
        self.rejected_reason = None

    def ingest(self, packet, now, grid=None):
        try:
            session = packet['sensor_session']; sequence = packet['sequence']; stamp = float(packet['time'])
            point = np.asarray(packet['sensor_position'], float); yaw = float(packet['sensor_yaw'])
            pose_time = float(packet['pose_time']); version = packet['map_version']
            shape = packet['shape']; origin = np.asarray(packet['origin'], float); resolution = float(packet['resolution'])
            if (packet['source'] != self.source or packet.get('sensor') != 'gazebo_gpu_lidar' or
                    packet.get('integration_completed') is not True or packet.get('ray_cell_count', 0) <= 0 or
                    packet.get('point_count', 0) <= 0 or not isinstance(session, str) or not session or
                    type(sequence) is not int or sequence < 1 or type(version) is not int or version < 0 or
                    point.shape != (3,) or origin.shape != (3,) or len(shape) != 3 or
                    any(type(v) is not int or v <= 0 for v in shape) or resolution <= 0 or
                    not np.isfinite([*point, *origin, yaw, pose_time, stamp, resolution]).all() or
                    abs(pose_time-stamp) > .15 or not -.1 <= now-stamp <= self.max_age_s):
                raise ValueError('invalid_sensor_frame')
            digest = hashlib.sha256(json.dumps([shape, origin.tolist(), resolution], separators=(',', ':')).encode()).hexdigest()
            if grid and digest != grid:
                raise ValueError('sensor_grid_mismatch')
            if session in self.retired_sessions or stamp <= self.last_time or (session == self.session and sequence <= self.sequence):
                raise ValueError('duplicate_or_late_frame')
            if session == self.session and self.frames and version < self.frames[-1]['map_version']:
                raise ValueError('map_version_regressed')
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.rejected_reason = str(exc); return False
        if self.session is not None and self.session != session:
            self.retired_sessions.add(self.session); self.frames.clear()
        self.session = session; self.sequence = sequence; self.last_time = stamp
        # Copy only the small completion proof, not thousands of map cells.
        self.frames.append(dict(sensor_session=session, sequence=sequence, time=stamp,
                                position=point.tolist(), yaw=yaw, map_version=version,
                                shape=shape, origin=origin.tolist(), resolution=resolution,
                                grid=digest, ray_cell_count=packet['ray_cell_count']))
        self.rejected_reason = None; return True

    def proof(self, now, arrived_at, goal, yaw, grid=None):
        if arrived_at is None:
            return None
        for frame in reversed(self.frames):
            angle = abs(math.atan2(math.sin(frame['yaw']-yaw), math.cos(frame['yaw']-yaw)))
            if (frame['time'] < arrived_at or not 0 <= now-frame['time'] <= self.max_age_s or
                    np.linalg.norm(np.asarray(frame['position'])-goal) >= .07 or angle >= .12):
                continue
            if grid and frame['grid'] != grid:
                continue
            return dict(frame, completion_reason='post_arrival_integrated_sensor_frame', arrived_at=arrived_at)
        return None
