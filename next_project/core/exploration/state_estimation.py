"""Error-state position/velocity/accelerometer-bias Kalman filter.

Orientation is supplied by an IMU attitude estimate. Absolute/SLAM odometry
measurements carry covariance; the filter never reads simulator geometry.
"""
import numpy as np


class InertialOdometryFilter:
    def __init__(self):
        self.x = np.zeros(9); self.P = np.eye(9); self.initialized = False
        self.time = None; self.updates = 0; self.rejected = 0

    def predict(self, stamp, world_acceleration):
        if self.time is None:
            self.time = stamp; return
        dt = stamp-self.time
        if dt <= 0:
            self.rejected += 1; return
        self.time = stamp
        if not self.initialized or dt > .2:
            self.P += np.eye(9)*.01; return
        F = np.eye(9); F[:3, 3:6] = np.eye(3)*dt
        F[:3, 6:] = -np.eye(3)*.5*dt*dt; F[3:6, 6:] = -np.eye(3)*dt
        a = np.asarray(world_acceleration)-self.x[6:]
        self.x[:3] += self.x[3:6]*dt+.5*a*dt*dt; self.x[3:6] += a*dt
        self.P = F@self.P@F.T+np.diag([1e-6]*3+[.002]*3+[1e-6]*3)*dt

    def correct(self, position, velocity, covariance):
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
