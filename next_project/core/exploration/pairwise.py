"""Capacity-constrained two-vehicle coverage routing and bilateral transactions.

Held-Karp computes open tours for every subset in a bounded interaction window.
The assignment step minimizes total travel with makespan pressure, with hard
unknown-volume capacities and pinned committed tasks. No external LKH process or
claim of global fleet optimality is required.
"""
import hashlib
import json
import uuid
import numpy as np


def fingerprint(owners):
    return hashlib.sha256(json.dumps(sorted((int(k), int(v)) for k, v in owners.items())).encode()).hexdigest()[:20]


def subset_tours(start, between):
    n = len(start); size = 1 << n
    dp = np.full((size, n), np.inf); prev = np.full((size, n), -1, int)
    for j in range(n):
        dp[1 << j, j] = start[j]
    for mask in range(1, size):
        for j in range(n):
            if not mask & (1 << j):
                continue
            before = mask ^ (1 << j)
            if not before:
                continue
            values = dp[before]+between[:, j]
            k = int(np.argmin(values)); dp[mask, j] = values[k]; prev[mask, j] = k
    costs = np.min(dp, axis=1) if n else np.zeros(1); costs[0] = 0.
    def route(mask):
        if not mask:
            return []
        j = int(np.argmin(dp[mask])); result = []
        while mask:
            result.append(j); k = prev[mask, j]; mask ^= 1 << j; j = k
        return result[::-1]
    return costs, route


def solve_pair(ids, start_costs, between, demands, owners, pair, pinned=None, capacity_factor=1.35, fixed_loads=(0., 0.)):
    ids = list(ids); n = len(ids)
    if n > 12:
        raise ValueError('Pair interaction window is bounded at 12 regions')
    pinned = pinned or {}; starts = np.array(start_costs, float); between = np.array(between, float)
    demands = np.asarray(demands, float); fixed_loads = np.asarray(fixed_loads, float)
    if starts.shape != (2, n) or between.shape != (n, n) or demands.shape != (n,) or np.any(demands < 0):
        raise ValueError('Invalid CVRP dimensions/demands')
    if fixed_loads.shape != (2,) or not np.isfinite(fixed_loads).all() or np.any(fixed_loads < 0):
        raise ValueError('Invalid frozen outside-window workload')
    tables = [subset_tours(starts[i], between) for i in range(2)]
    total_mask = (1 << n)-1
    volumes = np.zeros(1 << n)
    for mask in range(1, 1 << n):
        bit = mask & -mask; j = bit.bit_length()-1; volumes[mask] = volumes[mask ^ bit]+demands[j]
    forced = [sum(1 << j for j, rid in enumerate(ids) if pinned.get(rid) == i) for i in pair]
    cap = max(float(demands.sum()+fixed_loads.sum())*.5*capacity_factor,
              float(demands.max(initial=0))+float(fixed_loads.min()),
              *(volumes[m]+fixed_loads[i] for i, m in enumerate(forced)))
    def objective(mask):
        a = tables[0][0][mask]; b = tables[1][0][total_mask ^ mask]
        return a+b+.25*max(a, b)
    initial = sum(1 << j for j, rid in enumerate(ids) if owners.get(rid) == pair[0])
    best = None
    for mask in range(1 << n):
        other = total_mask ^ mask
        if mask & forced[0] != forced[0] or other & forced[1] != forced[1]:
            continue
        if volumes[mask]+fixed_loads[0] > cap+1e-8 or volumes[other]+fixed_loads[1] > cap+1e-8:
            continue
        value = objective(mask)
        if np.isfinite(value) and (best is None or (value, mask) < best):
            best = (float(value), mask)
    if best is None:
        return dict(status='infeasible', capacity=cap, assignments={}, routes={}, before=float(objective(initial)), after=None)
    value, mask = best
    assignments = {rid: pair[0] if mask & (1 << j) else pair[1] for j, rid in enumerate(ids)}
    routes = {str(pair[i]): [ids[j] for j in tables[i][1](mask if i == 0 else total_mask ^ mask)] for i in range(2)}
    return dict(status='optimal_window', assignments=assignments, routes=routes, capacity=cap,
                loads=[float(volumes[mask]+fixed_loads[0]), float(volumes[total_mask ^ mask]+fixed_loads[1])],
                fixed_loads=fixed_loads.tolist(),
                before_feasible=bool(max(volumes[initial]+fixed_loads[0], volumes[total_mask ^ initial]+fixed_loads[1]) <= cap+1e-8 and np.isfinite(objective(initial))),
                before=float(objective(initial)), after=value)


class PairExchange:
    """Prepare/accept/commit/applied handshake; one locked interaction per UAV.

    Routing ownership alone never authorizes motion; the independent reservation
    protocol protects transiently inconsistent replicas during commit delivery.
    """
    def __init__(self, drone, session=None):
        self.session = session or uuid.uuid4().hex
        self.drone = drone; self.counter = 0; self.transaction = None; self.overrides = {}
        self.completed = set(); self.commits = 0; self.last_success = {}; self.rejections = 0

    def offer(self, other, result, owners, now):
        if self.transaction or result['status'] != 'optimal_window' or (result.get('before_feasible', True) and result['after'] >= result['before']-.2):
            return False
        self.counter += 1
        ids = set(result['assignments']); base = {r: owners[r] for r in ids}
        self.transaction = dict(token=f'{self.drone}:{self.session[:8]}:{self.counter}', leader=self.drone, follower=other,
            phase='prepare', created=now, expires=now+12., base=fingerprint(base), ids=sorted(ids),
            assignments={str(k): v for k, v in result['assignments'].items()}, routes=result['routes'],
            before=float(result['before']) if np.isfinite(result['before']) else None, after=result['after'], capacity=result['capacity'], loads=result['loads'])
        return True

    def apply(self, tx):
        if tx['token'] not in self.completed:
            self.overrides.update({int(k): int(v) for k, v in tx['assignments'].items()})
            self.completed.add(tx['token']); self.commits += 1
            other = tx['leader'] if tx['leader'] != self.drone else tx['follower']
            self.last_success[other] = tx['created']

    def tick(self, peers, owners, now, pinned=None):
        pinned = pinned or {}
        tx = self.transaction
        if tx and now > tx['expires']:
            # Once committed, retain and retransmit until applied; do not undo a
            # decision another participant may already have installed.
            if tx['phase'] not in ('commit', 'applied'):
                self.transaction = None; tx = None
        if tx:
            other = tx['follower'] if tx['leader'] == self.drone else tx['leader']
            packet = peers.get(other, {}).get('pair_transaction')
            if packet and packet['token'] == tx['token']:
                if tx['leader'] == self.drone and tx['phase'] == 'prepare' and packet['phase'] == 'accept':
                    tx['phase'] = 'commit'
                elif tx['leader'] != self.drone and tx['phase'] == 'accept' and packet['phase'] == 'commit':
                    self.apply(tx); tx['phase'] = 'applied'
                elif tx['leader'] == self.drone and tx['phase'] == 'commit' and packet['phase'] == 'applied':
                    self.apply(tx); tx['phase'] = 'applied'; tx['expires'] = now+2.
                elif tx['phase'] == 'applied' and packet['phase'] == 'applied':
                    self.transaction = None
            if tx['phase'] == 'applied' and now > tx['expires']+2:
                self.transaction = None
            return
        for source, peer in sorted(peers.items()):
            candidate = peer.get('pair_transaction')
            if not candidate or candidate['phase'] != 'prepare' or candidate['follower'] != self.drone:
                continue
            if candidate['token'] in self.completed or now > candidate['expires']:
                continue
            ids = candidate['ids']; base = {r: owners.get(r, -1) for r in ids}
            assignments = {int(k): int(v) for k, v in candidate['assignments'].items()}
            valid = (candidate['leader'] == source and fingerprint(base) == candidate['base'] and
                     set(assignments) == set(ids) and all(v in (source, self.drone) for v in assignments.values()) and
                     all(assignments.get(r, owner) == owner for r, owner in pinned.items()) and
                     max(candidate['loads'], default=0) <= candidate['capacity']+1e-8)
            if valid:
                self.transaction = dict(candidate, phase='accept'); return
            self.rejections += 1
