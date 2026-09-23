"""Peer map replication, route-insertion auction and unanimous path leases.

There is no elected or permanent allocation server. Each participant runs this
state machine. Fixed fleet membership deliberately sacrifices availability under
partition: no new path may commit without every participant's reservation ACK.
"""
import numpy as np
from scipy.spatial import cKDTree
from .mapping import ObservedMap


class MapReplica:
    def __init__(self, bounds, drone, volumetric=False, flight_limits=None):
        from .voxel_mapping import VoxelMap
        self.drone = drone; self.map = VoxelMap(bounds, flight_limits=flight_limits) if volumetric else ObservedMap(bounds)
        self.stamps = np.full(self.map.shape, -1., float)
        self.sources = np.full(self.map.shape, -1, np.int8)
        self.sequences = {}; self.sensor_sessions = {}; self.retired_sensors = {}; self.sensor_stamps = {}

    def merge(self, packet):
        source = int(packet['source']); sequence = int(packet['sequence'])
        session = packet.get('sensor_session')
        if session in self.retired_sensors.get(source, set()):
            return False
        restarting = source in self.sensor_sessions and self.sensor_sessions[source] != session
        if restarting:
            if float(packet['time']) < self.sensor_stamps.get(source, -1.):return False
        if sequence <= (-1 if restarting else self.sequences.get(source, -1)):
            return False
        indices = np.asarray(packet['indices'], int).reshape(-1, self.map.state.ndim)
        values = np.asarray(packet['values'], np.int8)
        if len(indices) != len(values) or np.any(indices < 0) or np.any(indices >= self.map.shape):
            raise ValueError('Invalid observation coordinates')
        if not np.isin(values, [0, 1]).all():
            raise ValueError('Observation must contain free/occupied states')
        stamp = float(packet['time'])
        if not np.isfinite(stamp):
            raise ValueError('Observation timestamp must be finite')
        if restarting:
            self.retired_sensors.setdefault(source, set()).add(self.sensor_sessions[source])
        self.sequences[source] = sequence
        self.sensor_sessions[source] = session; self.sensor_stamps[source] = stamp
        key = tuple(indices.T)
        accept = (stamp > self.stamps[key]) | ((stamp == self.stamps[key]) & (source > self.sources[key]))
        changed = self.map.update(indices[accept], values[accept]) if accept.any() else False
        self.stamps[tuple(indices[accept].T)] = stamp
        self.sources[tuple(indices[accept].T)] = source
        return changed


def sample_path(path, step=.2):
    p = np.asarray(path, float).reshape(-1, 3)
    return np.vstack([p]+[np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b-a)/step))+1)) for a, b in zip(p[:-1], p[1:])])


def paths_conflict(a, b, radius=1.2):
    if not len(a) or not len(b):
        return False
    return bool(cKDTree(sample_path(a)[:, :2]).query(sample_path(b)[:, :2])[0].min() < radius)


def remainder(position, path):
    p = np.asarray(path, float)
    if len(p) <= 1:
        return np.array([position, p[-1]])
    a, d = p[:-1], np.diff(p, axis=0)
    u = np.clip(np.sum((position-a)*d, axis=1)/np.maximum(np.sum(d*d, axis=1), 1e-12), 0, 1)
    projections = a+u[:, None]*d
    k = int(np.argmin(np.linalg.norm(projections-position, axis=1)))
    return np.vstack([position, projections[k], p[k+1:]])


def regions_conflict(a, b):
    if a['region'] == b['region']:
        return True
    if a.get('bounds') is None or b.get('bounds') is None:
        return False
    aa, bb = np.asarray(a['bounds']), np.asarray(b['bounds'])
    return bool(aa.shape == bb.shape and np.all(np.minimum(aa[1], bb[1])-np.maximum(aa[0], bb[0]) > 1e-6))


class PeerLedger:
    def __init__(self, drone, members=(0, 1, 2), timeout=3.):
        self.drone = drone; self.members = tuple(members); self.timeout = timeout; self.seeds = None
        self.states = {}; self.grants = {}; self.retired_sessions = {}

    def receive(self, state):
        source = int(state['drone'])
        if source not in self.members or source == self.drone:
            return False
        previous = self.states.get(source)
        old_session = previous.get('session') if previous else None
        session = state.get('session')
        if session in self.retired_sessions.get(source, set()):
            return False
        if previous and session != old_session:
            if not (state.get('rejoin_ready') and state.get('stopped') and state.get('epoch', 0) > previous.get('epoch', 0) and state['time'] >= previous['time']):
                return False
            if old_session is not None:
                self.retired_sessions.setdefault(source, set()).add(old_session)
            previous = None
        if previous and state['sequence'] <= previous['sequence']:
            return False
        self.states[source] = state
        return True

    def fresh(self, now):
        return all(i in self.states and -.1 <= now-self.states[i]['time'] < self.timeout for i in self.members if i != self.drone)

    def owners(self, own, now):
        peers = [own]+[p for p in self.states.values() if -.1 <= now-p['time'] < self.timeout]
        winners = {}
        for p in peers:
            if not p.get('available', False):
                continue
            for rid, bid in p.get('bids', {}).items():
                value = (float(bid), int(p['drone']))
                if int(rid) not in winners or value < winners[int(rid)]:
                    winners[int(rid)] = value
        return {rid: value[1] for rid, value in winners.items()}

    def acknowledge(self, own, now):
        """Grant paths atomically against our committed/pending path and grants.

        Grants remain reserved until explicit withdrawal, including during silence. A participant
        cannot grant intersecting intents and cannot propose across its grants.
        """
        live = {p['intent']['token']: p for p in self.states.values()
                if p.get('intent')}
        self.grants = {k: v for k, v in self.grants.items() if k in live}
        my_intent = own.get('intent')
        for token, peer in sorted(live.items(), key=lambda kv: (kv[1]['intent']['created'], kv[1]['drone'])):
            if token in self.grants:
                # Refresh a moving path's consumed prefix, never change the lease ID.
                self.grants[token] = peer['intent']; continue
            intent = peer['intent']
            if not -.1 <= now-peer['time'] < self.timeout:
                continue
            if intent.get('contingency') and (self.seeds is None or not inside_contingency_cell(intent['path'], peer['drone'], self.seeds)):
                continue
            if paths_conflict(intent['path'], [own['position']]):
                continue
            if my_intent and (regions_conflict(my_intent, intent) or paths_conflict(my_intent['path'], intent['path'])):
                if my_intent.get('committed') or (my_intent['created'], self.drone) < (intent['created'], peer['drone']):
                    continue
                # Own losing proposal must be withdrawn before acknowledging.
                continue
            if any(regions_conflict(g, intent) or paths_conflict(g['path'], intent['path']) for g in self.grants.values()):
                continue
            self.grants[token] = intent
        return sorted(self.grants)

    def can_propose(self, path, region, now):
        if not self.fresh(now):
            if self.seeds is None or len(self.states) != len(self.members)-1 or not inside_contingency_cell(path, self.drone, self.seeds):
                return False
        for p in self.states.values():
            if paths_conflict(path, [p['position']]):
                return False
            intent = p.get('intent')
            if intent and (intent['region'] == region or paths_conflict(path, intent['path'])):
                return False
        return not any(g['region'] == region or paths_conflict(path, g['path']) for g in self.grants.values())

    def quorum(self, token, now, voters=None):
        if voters is None:
            return self.fresh(now) and all(token in p.get('acks', []) for p in self.states.values())
        return all(i in self.states and -.1 <= now-self.states[i]['time'] < self.timeout and token in self.states[i].get('acks', []) for i in voters)

    def loses(self, intent):
        for p in self.states.values():
            other = p.get('intent')
            if other and (regions_conflict(other, intent) or paths_conflict(other['path'], intent['path'])):
                if other.get('committed') or (other['created'], p['drone']) < (intent['created'], self.drone):
                    return True
        return False


def tracking_recovery(runtime, position, max_distance=.65, yaw=None):
    """Return to the nominal planning envelope through observed safe space.

    0.50 m exceeds the 0.453 m horizontal hull circumradius. It is used only for a short
    recovery connector, never to explore unknown cells or cross a narrow door.
    """
    import copy
    relaxed = copy.copy(runtime); relaxed.clearance = .5
    if runtime.state.ndim == 3 and yaw is not None:
        relaxed.recovery_yaw = yaw
    points = runtime.points(np.argwhere(runtime.safe))
    if not len(points) or not relaxed.safe_path([position]):
        return None
    for k in np.argsort(np.linalg.norm(points-position, axis=1))[:30]:
        if np.linalg.norm(points[k]-position) > max_distance:
            break
        path = np.array([position, points[k]])
        if relaxed.safe_path(path):
            return path
    return None


def execution_lease(states, drone, token, now, members=(0, 1, 2), timeout=3.):
    """Executor-side guard, independent of the planner's commit decision."""
    if token is None or any(i not in states or not -.1 <= now-states[i]['time'] < timeout for i in members):
        return False
    intent = states[drone].get('intent') or {}
    return bool(intent.get('token') == token and intent.get('committed') and not intent.get('retiring') and
                all(token in states[i].get('acks', []) for i in members if i != drone))


def inside_contingency_cell(path, drone, seeds, margin=1.25):
    """Pre-agreed disjoint fallback cells, inset by vehicle separation margin.

    Bisector half-spaces are convex, so checking polyline endpoints certifies its
    entire segments. Old reservations are additionally retained during outages.
    """
    p = np.asarray(path, float); seeds = np.asarray(seeds, float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        return False
    home = seeds[drone]
    for i, other in enumerate(seeds):
        if i == drone:
            continue
        delta = other-home; norm = np.linalg.norm(delta)
        if norm < 1e-6 or np.any((p-(home+other)/2)@(delta/norm) > -margin):
            return False
    return True


def fused_execution_lease(states, drone, token, now, seeds, timeout=3.):
    """Finish an already certified spatial reservation through packet outages.

    Peers never expire a reservation merely due to silence. New partition-mode
    routes additionally stay inside disjoint contingency cells and obtain ACKs
    from their declared live voters. No unseen peer is silently removed.
    """
    own = states.get(drone, {})
    if token is None or not -.1 <= now-own.get('time', -1e9) < timeout:
        return False
    intent = own.get('intent') or {}
    if intent.get('token') != token or not intent.get('committed') or intent.get('retiring'):
        return False
    voters = intent.get('voters', [i for i in range(len(seeds)) if i != drone])
    if intent.get('contingency') and not inside_contingency_cell(intent['path'], drone, seeds):
        return False
    return all(token in states.get(i, {}).get('acks', []) for i in voters)
