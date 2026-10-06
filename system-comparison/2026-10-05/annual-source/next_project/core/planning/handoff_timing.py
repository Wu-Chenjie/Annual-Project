"""Bounded wall-clock latency samples and an explicit simulation-clock conversion."""
from collections import deque
import math
import numpy as np


class HandoffTiming:
    def __init__(self, window=64, minimum_samples=5, planning_default_s=12., authorization_default_s=2.4, margin_wall_s=.5):
        if window < minimum_samples or minimum_samples < 1 or min(planning_default_s, authorization_default_s, margin_wall_s) < 0:
            raise ValueError('Invalid handoff timing configuration')
        self.window = window; self.minimum_samples = minimum_samples
        self.planning_default_s = planning_default_s; self.authorization_default_s = authorization_default_s
        self.margin_wall_s = margin_wall_s
        self.planning = deque(maxlen=window); self.authorization = deque(maxlen=window)
        self.rates = deque(maxlen=window); self.clock = None; self.authorizing = {}

    def observe_clock(self, simulation, wall):
        if not math.isfinite(simulation) or not math.isfinite(wall):
            return
        if self.clock is not None:
            ds, dw = simulation-self.clock[0], wall-self.clock[1]
            if ds < 0 or dw < 0:
                self.rates.clear(); self.clock = (simulation, wall); return
            if dw < .1:
                return
            self.rates.append(ds/dw)
        self.clock = (simulation, wall)

    def record_planning(self, wall_s):
        if math.isfinite(wall_s) and wall_s >= 0:
            self.planning.append(float(wall_s))

    def proposed(self, token, wall):
        # At most a current and a pending lease are normally authorizing.
        if len(self.authorizing) >= self.window:
            self.authorizing.pop(next(iter(self.authorizing)))
        self.authorizing.setdefault(token, wall)

    def authorized(self, token, wall):
        begin = self.authorizing.pop(token, None)
        if begin is not None and math.isfinite(wall-begin) and wall >= begin:
            self.authorization.append(wall-begin)

    def cancelled(self, token):
        self.authorizing.pop(token, None)

    def estimate(self):
        def latency(samples, default):
            # Include slow censored requests; a short history must not reduce
            # the conservative default below an already observed slow sample.
            if len(samples) < self.minimum_samples:
                return max(default, max(samples, default=0.)), 'conservative_default'
            return float(np.quantile(samples, .95)), 'rolling_p95'
        plan, plan_source = latency(self.planning, self.planning_default_s)
        auth, auth_source = latency(self.authorization, self.authorization_default_s)
        positive = [v for v in self.rates if v > 0]
        rate = float(np.quantile(positive, .95)) if positive else 1.
        wall = plan+auth+self.margin_wall_s
        return dict(window_size=self.window, minimum_samples=self.minimum_samples,
                    planning_samples=len(self.planning), authorization_samples=len(self.authorization),
                    planning_wall_p95_s=plan, planning_source=plan_source,
                    authorization_wall_p95_s=auth, authorization_source=auth_source,
                    margin_wall_s=self.margin_wall_s, lead_wall_s=wall,
                    real_time_factor=rate, clock_samples=len(self.rates),
                    clock_source='measured_p95' if positive else 'conservative_default',
                    lead_sim_s=max(.5, wall*rate))
