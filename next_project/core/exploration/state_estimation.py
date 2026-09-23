"""Error-state position/velocity/accelerometer-bias Kalman filter.

Orientation is supplied by an IMU attitude estimate. Absolute/SLAM odometry
measurements carry covariance; the filter never reads simulator geometry.
"""
import numpy as np


class InertialOdometryFilter:
    def __init__(self):
        self.x = np.zeros(9); self.P = np.eye(9); self.initialized = False
        self.time = None; self.updates = 0; self.rejected = 0
        self.history = []; self.anchor = None; self.measurement_time = None

    def predict(self, stamp, world_acceleration):
        acceleration = np.asarray(world_acceleration, float)
        if (not np.isfinite(stamp) or acceleration.shape != (3,) or not np.isfinite(acceleration).all()
                or self.time is not None and stamp <= self.time):
            self.rejected += 1; return False
        self._predict(stamp, acceleration)
        if self.initialized and (self.anchor is None or self.anchor[0] is None):
            self.anchor = (stamp, self.x.copy(), self.P.copy())
        self.history.append((stamp, acceleration.copy(), self.x.copy(), self.P.copy()))
        if len(self.history) > 200:
            t, _, x, P = self.history.pop(0)
            self.anchor = (t, x, P)
        return True

    def _predict(self, stamp, world_acceleration):
        if self.time is None:
            self.time = stamp; return
        dt = stamp-self.time
        if dt <= 0:return
        self.time = stamp
        if not self.initialized or dt > .2:
            self.P += np.eye(9)*.01; return
        F = np.eye(9); F[:3, 3:6] = np.eye(3)*dt
        F[:3, 6:] = -np.eye(3)*.5*dt*dt; F[3:6, 6:] = -np.eye(3)*dt
        a = np.asarray(world_acceleration)-self.x[6:]
        self.x[:3] += self.x[3:6]*dt+.5*a*dt*dt; self.x[3:6] += a*dt
        self.P = F@self.P@F.T+np.diag([1e-6]*3+[.002]*3+[1e-6]*3)*dt

    def correct(self, position, velocity, covariance):
        accepted = self._correct(position, velocity, covariance)
        if accepted:
            self.anchor = (self.time, self.x.copy(), self.P.copy()); self.history = []
        return accepted

    def correct_at(self, stamp, position, velocity, covariance):
        """Update at the measurement timestamp, then replay buffered IMU inputs.

        ROS delivery order is not sensor time order. Comparing a delayed pose or
        velocity directly with the current state can trip the innovation gate
        throughout takeoff. The bounded lag buffer never reads simulator truth.
        """
        if (self.time is None or not np.isfinite(stamp) or stamp > self.time+1e-8
                or self.measurement_time is not None and stamp <= self.measurement_time
                or self.anchor is not None and self.anchor[0] is not None and stamp < self.anchor[0]-1e-8):
            self.rejected += 1; return False
        old = (self.time, self.x.copy(), self.P.copy(), self.initialized)
        inputs = [(t, a) for t, a, _, _ in self.history]
        future = [(t, a) for t, a in inputs if t > stamp+1e-8]
        if self.initialized:
            if self.anchor is None or self.anchor[0] is None:
                self.rejected += 1; return False
            self.time, x, P = self.anchor; self.x = x.copy(); self.P = P.copy()
            for t, a in inputs:
                if t <= stamp+1e-8:self._predict(t, a)
            if self.time < stamp:
                self._predict(stamp, future[0][1] if future else inputs[-1][1] if inputs else np.zeros(3))
        else:
            self.time = stamp
        if not self._correct(position, velocity, covariance):
            self.time, self.x, self.P, self.initialized = old
            return False
        self.anchor = (stamp, self.x.copy(), self.P.copy()); self.measurement_time = stamp
        self.history = []
        for t, a in future:
            self._predict(t, a)
            self.history.append((t, a.copy(), self.x.copy(), self.P.copy()))
        return True

    def _correct(self, position, velocity, covariance):
        z = np.r_[position, velocity]; R = np.asarray(covariance, float)
        if not np.isfinite(z).all() or R.shape != (6, 6) or np.any(np.linalg.eigvalsh(R) <= 0):
            self.rejected += 1; return False
        if not self.initialized:
            self.x[:6] = z; self.P[:6, :6] = R; self.P[6:, 6:] *= .01; self.initialized = True
        else:
            H = np.c_[np.eye(6), np.zeros((6, 3))]
            innovation = z-H@self.x; S = H@self.P@H.T+R
            if innovation@np.linalg.solve(S, innovation) > 40.:
                self.rejected += 1; return False
            K = np.linalg.solve(S, H@self.P).T
            self.x += K@innovation
            I = np.eye(9)-K@H
            self.P = I@self.P@I.T+K@R@K.T
        self.updates += 1; return True
