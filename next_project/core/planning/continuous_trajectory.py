"""Minimum-jerk quintic spline with optimized segment times and hard validation.

A C4 quintic interpolant with prescribed endpoint velocity/acceleration is the
minimum integrated squared jerk curve for fixed waypoint times. SciPy's banded
B-spline interpolation solves that inner problem. The outer problem optimizes
positive segment times. This is a small reference implementation of minimum
control interpolation, not a vendored GCOPTER/MINCO implementation.
"""
import math
import time
import numpy as np
from scipy.interpolate import make_interp_spline, PPoly
from scipy.optimize import minimize


def derivative(coefficients, order):
    c = np.asarray(coefficients)
    for _ in range(order):
        c = c[1:]*np.arange(1, len(c))[:, None]
    return c


def evaluate_polynomial(c, u):
    return np.polynomial.polynomial.polyval(u, c).T


def maximum_norm(c):
    squared = np.zeros(2*len(c)-1)
    for d in range(c.shape[1]):
        term = np.polynomial.polynomial.polymul(c[:, d], c[:, d])
        squared[:len(term)] += term
    roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(squared))
    candidates = [0., 1.]+[float(r.real) for r in roots if abs(r.imag) < 1e-7 and 0 < r.real < 1]
    return float(np.sqrt(max(0., max(np.polynomial.polynomial.polyval(candidates, squared)))))


class ContinuousTrajectory:
    def __init__(self, durations, coefficients, yaw, yaw_delta, method='minimum_jerk_bspline', yaw_coefficients=None):
        self.durations = np.asarray(durations, float); self.coefficients = np.asarray(coefficients, float)
        self.knots = np.r_[0., np.cumsum(self.durations)]; self.duration = float(self.knots[-1])
        self.yaw = float(yaw); self.yaw_delta = float(yaw_delta); self.method = method
        self.yaw_coefficients = np.array([yaw, 0., 0., 10*yaw_delta, -15*yaw_delta, 6*yaw_delta]
                                        if yaw_coefficients is None else yaw_coefficients, float)
        if (len(self.durations) == 0 or np.any(self.durations <= 0) or not np.isfinite(self.durations).all()
                or self.coefficients.shape != (len(self.durations), 6, 3) or not np.isfinite(self.coefficients).all()
                or not np.isfinite([self.yaw, self.yaw_delta]).all() or self.yaw_coefficients.shape != (6,)
                or not np.isfinite(self.yaw_coefficients).all()):
            raise ValueError('Malformed continuous trajectory')

    def sample(self, t):
        t = float(np.clip(t, 0., self.duration)); i = min(len(self.durations)-1, np.searchsorted(self.knots, t, side='right')-1)
        u = (t-self.knots[i])/self.durations[i]; c = self.coefficients[i]; T = self.durations[i]
        p, v, a = [evaluate_polynomial(derivative(c, k), u)/T**k for k in range(3)]
        z = t/self.duration
        yaw = float(np.polynomial.polynomial.polyval(z, self.yaw_coefficients))
        rate = float(np.polynomial.polynomial.polyval(z, np.polynomial.polynomial.polyder(self.yaw_coefficients)))/self.duration
        return p, v, a, yaw, rate

    def limits(self):
        return {name: max(maximum_norm(derivative(c, k))/T**k for c, T in zip(self.coefficients, self.durations))
                for k, name in [(1, 'speed'), (2, 'acceleration'), (3, 'jerk')]}

    def jerk_cost(self):
        cost = 0.
        for c, T in zip(self.coefficients, self.durations):
            j = derivative(c, 3)
            for axis in range(3):
                poly = np.polynomial.polynomial.polymul(j[:, axis], j[:, axis])
                cost += float(sum(poly/np.arange(1, len(poly)+1)))/T**5
        return cost

    def yaw_acceleration(self, t):
        return float(np.polynomial.polynomial.polyval(np.clip(t/self.duration, 0., 1.),
                     np.polynomial.polynomial.polyder(self.yaw_coefficients, 2)))/self.duration**2

    def yaw_rate_limit(self):
        return maximum_norm(np.polynomial.polynomial.polyder(self.yaw_coefficients)[:, None])/self.duration

    def path(self, dt=.08):
        return np.array([self.sample(t)[0] for t in np.linspace(0, self.duration, max(2, int(np.ceil(self.duration/dt))+1))])

    def to_dict(self):
        return dict(schema='annual.trajectory/1', durations=self.durations.tolist(), coefficients=self.coefficients.tolist(),
                    yaw=self.yaw, yaw_delta=self.yaw_delta, method=self.method, duration=self.duration,
                    yaw_coefficients=self.yaw_coefficients.tolist(), yaw_rate_limit=self.yaw_rate_limit(),
                    limits=self.limits(), jerk_integral=self.jerk_cost())

    @classmethod
    def from_dict(cls, packet):
        if packet.get('schema') != 'annual.trajectory/1':
            raise ValueError('Unsupported trajectory schema')
        result = cls(packet['durations'], packet['coefficients'], packet['yaw'], packet['yaw_delta'], packet['method'], packet.get('yaw_coefficients'))
        if result.limits()['speed'] > .601 or result.limits()['acceleration'] > .801 or result.limits()['jerk'] > 2.001 or result.yaw_rate_limit() > .651:
            raise ValueError('Trajectory exceeds executor dynamics envelope')
        return result


def interpolate(points, durations, yaw, delta, start_velocity=None, start_acceleration=None, start_yaw_rate=0., start_yaw_acceleration=0.):
    times = np.r_[0., np.cumsum(durations)]; zero = np.zeros(3)
    velocity = zero if start_velocity is None else np.asarray(start_velocity, float)
    acceleration = zero if start_acceleration is None else np.asarray(start_acceleration, float)
    spline = make_interp_spline(times, points, k=5, bc_type=([(1, velocity), (2, acceleration)], [(1, zero), (2, zero)]))
    coefficients = np.zeros((len(durations), 6, 3))
    for axis in range(3):
        p = PPoly.from_spline((spline.t, spline.c[:, axis], spline.k))
        for i, (start, T) in enumerate(zip(times, durations)):
            j = np.searchsorted(p.x, start+T*.5)-1
            coefficients[i, :, axis] = p.c[:, j][::-1]*T**np.arange(6)
    T = times[-1]; c = np.array([yaw, start_yaw_rate*T, .5*start_yaw_acceleration*T*T, 0., 0., 0.])
    c[3:] = np.linalg.solve([[1, 1, 1], [3, 4, 5], [6, 12, 20]],
                            [delta-c[1]-c[2], -c[1]-2*c[2], -2*c[2]])
    return ContinuousTrajectory(durations, coefficients, yaw, delta, yaw_coefficients=c)


def simplify_path(path, runtime):
    p = np.asarray(path, float); p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-6]]
    if len(p) == 1:
        return np.vstack([p, p])
    result = [p[0]]; k = 0
    while k < len(p)-1:
        j = len(p)-1
        while j > k+1 and not runtime.safe_path([p[k], p[j]]):
            j -= 1
        result.append(p[j]); k = j
    return np.array(result)


def optimize_trajectory(path, runtime, yaw, target_yaw, speed_limit=.6, acceleration_limit=.8,
                        start_velocity=None, start_acceleration=None, start_yaw_rate=0., start_yaw_acceleration=0.,
                        deadline_wall=None):
    def check_budget():
        if deadline_wall is not None and time.monotonic() >= deadline_wall:
            raise TimeoutError('Trajectory fitting wall budget exhausted')
    check_budget()
    if not runtime.safe_path(path):
        raise ValueError('Unsafe input route')
    points = simplify_path(path, runtime)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    delta = math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw))
    initial = np.maximum(distances/.4, .6)
    boundary = dict(start_velocity=start_velocity, start_acceleration=start_acceleration,
                    start_yaw_rate=start_yaw_rate, start_yaw_acceleration=start_yaw_acceleration)
    moving = any(x is not None and np.linalg.norm(x) > 1e-8 for x in boundary.values())
    def objective(log_times):
        check_budget()
        trajectory = interpolate(points, np.exp(log_times), yaw, delta, **boundary)
        times = np.linspace(0., trajectory.duration, 40)
        samples = [trajectory.sample(t) for t in times]
        speed = max(np.linalg.norm(s[1]) for s in samples)
        accel = max(np.linalg.norm(s[2]) for s in samples)
        collision = sum(max(0., runtime.clearance-runtime.signed_distance(s[0]))**2 for s in samples)
        return trajectory.duration+.035*trajectory.jerk_cost()+3000*(max(speed-.6, 0)**2+max(accel-.8, 0)**2+collision)
    optimized = minimize(objective, np.log(initial), method='L-BFGS-B',
                         bounds=[(np.log(max(.4, d/.7)), np.log(max(3., d/.15))) for d in distances],
                         options=dict(maxiter=10, maxfun=160, ftol=1e-4))
    check_budget()
    trajectory = interpolate(points, np.exp(optimized.x), yaw, delta, **boundary)
    if not runtime.safe_path(trajectory.path()):
        if moving:
            raise ValueError('Moving boundary curve leaves the certified corridor')
        # Conservative exact C2 fallback stays on the already checked polyline.
        # It is explicitly logged, never labeled as the optimized spline.
        coeff = np.zeros((len(distances), 6, 3)); coeff[:, 0] = points[:-1]
        d = np.diff(points, axis=0); coeff[:, 3] = 10*d; coeff[:, 4] = -15*d; coeff[:, 5] = 6*d
        trajectory = ContinuousTrajectory(initial, coeff, yaw, delta, 'stop_at_corner_quintic')
    limits = trajectory.limits()
    scale = max(1., limits['speed']/speed_limit, np.sqrt(limits['acceleration']/acceleration_limit), np.cbrt(limits['jerk']/2.),
                1.875*abs(delta)/(.65*trajectory.duration))*1.015
    if moving:
        # Refit, rather than rescale coefficients: scaling changes boundary v/a.
        for _ in range(8):
            limits = trajectory.limits()
            factor = max(1., limits['speed']/speed_limit, np.sqrt(limits['acceleration']/acceleration_limit),
                         np.cbrt(limits['jerk']/2.), trajectory.yaw_rate_limit()/.65)
            if factor <= 1.001: break
            trajectory = interpolate(points, trajectory.durations*min(factor*1.02, 1.5), yaw, delta, **boundary)
        limits = trajectory.limits()
        if limits['speed'] > speed_limit+.001 or limits['acceleration'] > acceleration_limit+.001 or limits['jerk'] > 2.001 or trajectory.yaw_rate_limit() > .651:
            raise ValueError('No feasible moving boundary within dynamics limits')
    else:
        trajectory = ContinuousTrajectory(trajectory.durations*scale, trajectory.coefficients, yaw, delta, trajectory.method)
    if not runtime.safe_path(trajectory.path()):
        raise ValueError('Continuous curve collision validation failed')
    return trajectory
