"""Team information accounting, separate from private collision geometry.

Only actually received observed-cell receipts enter knowledge. Future promises
may reduce estimated utility but never complete a region or open a safe voxel.
"""
import hashlib
import numpy as np


def known_mask(runtime, observed_mask=None):
    local = (runtime.state != -1).ravel()
    if observed_mask is None:
        return local.copy()
    mask = np.asarray(observed_mask, dtype=bool).ravel()
    if mask.shape != local.shape:
        raise ValueError('Team evidence grid does not match the private map')
    return local | mask


def region_cells(runtime, bounds):
    low, high = np.asarray(bounds, float)[:, :runtime.state.ndim]
    a = np.maximum(0, np.floor((low-runtime.origin)/runtime.resolution).astype(int))
    b = np.minimum(runtime.shape, np.ceil((high-runtime.origin)/runtime.resolution).astype(int))
    return tuple(slice(int(i), int(j)) for i, j in zip(a, b))


def regional_evidence(runtime, bounds, observed_mask=None):
    window = region_cells(runtime, bounds)
    known = known_mask(runtime, observed_mask).reshape(runtime.shape)[window]
    local = runtime.state[window]
    digest = hashlib.sha256(local.tobytes()+np.packbits(known.ravel()).tobytes()).hexdigest()[:20]
    return dict(local_unknown=int(np.count_nonzero(local == -1)),
                team_unknown=int(np.count_nonzero(~known)), cells=int(known.size),
                known_ratio=float(known.mean()) if known.size else 1., signature=digest)


def team_new_cells(cells, observed_mask):
    if not cells:
        return set()
    ids = np.fromiter(cells, dtype=int)
    return set(map(int, ids[~np.asarray(observed_mask, bool).ravel()[ids]]))


def service_accounting(runtime, cells, team_before, peer_receipts_at_end=None, first_observed=None,start=None,end=None):
    """Online novelty estimate against receipts available at service start.

Late / concurrent peer observations are attributed exactly only by the offline
sensor auditor. Do not subtract our own just-published receipt at service end.
"""
    ids = np.asarray(cells, dtype=int)
    measured = runtime.state.ravel()[ids] != -1
    if first_observed is not None:
        times=np.asarray(first_observed).ravel()[ids]
        measured &= (times>start)&(times<=end)
    seen = np.asarray(team_before, bool).ravel()
    if peer_receipts_at_end is not None:
        seen = seen | np.asarray(peer_receipts_at_end, bool).ravel()
    team_new = measured & ~seen[ids]
    return dict(local_new_cells=int(measured.sum()), team_new_cells=int(team_new.sum()),
                local_free_cells=int(np.count_nonzero(measured & (runtime.state.ravel()[ids] == 0))),
                team_free_cells=int(np.count_nonzero(team_new & (runtime.state.ravel()[ids] == 0))))


def may_reactivate(feedback, evidence, now, predicted_cells=0, verified_alternative=False):
    if not feedback or not feedback.get('low_yield_streak'):
        return True, 'new_or_successful_task'
    if now < feedback.get('defer_until', 0.):
        return False, 'cooldown'
    previous = feedback.get('evidence_signature')
    if previous is None:
        return True, 'legacy_feedback_recheck'
    if evidence['signature'] != previous:
        return True, 'actual_evidence_changed'
    if verified_alternative and predicted_cells >= 1:
        return True, 'different_view_with_verified_ray_gain'
    # Expiry requests a re-evaluation. Only a proven improvement can authorize
    # another unchanged exploration service; waiting alone cannot do that.
    improved = predicted_cells > max(1, feedback.get('expected_team_cells', 0))*1.25
    return bool(improved), 'gain_improved' if improved else 'unchanged_low_yield'


def intent_records(peer):
    return [peer[key] for key in ('intent', 'pending_intent') if peer.get(key)]+list(peer.get('retiring_intents', []))


def predicted_peer_completion(peer, intent, now):
    execution = peer.get('execution', {})
    if execution.get('token') is not None and execution.get('token') == intent.get('token'):
        return now+max(0., execution.get('trajectory_duration', 0.)-execution.get('trajectory_time', 0.))+.65
    boundary = intent.get('handoff') or {}
    if execution.get('token') is not None and execution.get('token') == boundary.get('from_token') and boundary:
        return now+max(0., boundary['trajectory_time']-execution.get('trajectory_time', 0.))+intent.get('duration',0.)+.65
    return max(now, intent.get('committed_at', intent.get('created', now))+intent.get('duration', 0.)+.65)


def expected_traffic_delay(path, peers, now, speed=.6, diagnostic_radius=2.):
    """Soft corridor contention estimate; never replaces hard reservations."""
    points = np.asarray(path, float)
    if len(points) < 2:
        return 0.
    # Arc-length samples make the estimate independent of path vertex density.
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]; total = cumulative[-1]
    if total < 1e-8: return 0.
    n = max(1, int(np.ceil(total/.3-1e-10)))
    stations = (np.arange(n)+.5)*total/n
    points = np.column_stack([np.interp(stations,cumulative,points[:,k]) for k in range(points.shape[1])])
    weights = np.full(n,total/n/speed); arrivals = stations/speed
    delay = 0.
    for peer in peers.values():
        if not -.1 <= now-peer.get('time', -np.inf) < 3.:
            continue
        peer_delay = 0.
        for intent in intent_records(peer):
            if not intent.get('committed') or intent.get('retiring'):
                continue
            other = np.asarray(intent.get('path', []), float)
            if not len(other): continue
            # The consumed prefix no longer occupies a corridor. Pending
            # routes keep their complete footprint until actual consumption.
            if peer.get('execution', {}).get('token') == intent.get('token') and peer.get('position') is not None:
                index = int(np.argmin(np.linalg.norm(other-np.asarray(peer['position']), axis=1)))
                other = np.vstack([peer['position'], other[index:]])
            # Distance to segments, rather than vertices, catches a long
            # two-vertex corridor crossing just as a dense sampled route does.
            if len(other) == 1:
                distance = np.linalg.norm(points[:, :2]-other[0, :2], axis=1)
            else:
                a = other[:-1, :2]; delta = np.diff(other[:, :2], axis=0)
                projection = np.sum((points[:, None, :2]-a[None])*delta[None], axis=2)/np.maximum(np.sum(delta*delta, axis=1), 1e-12)
                nearest = a[None]+np.clip(projection, 0., 1.)[:, :, None]*delta[None]
                distance = np.linalg.norm(points[:, None, :2]-nearest, axis=2).min(axis=1)
            remaining = max(0., predicted_peer_completion(peer, intent, now)-now)
            overlap = (distance < diagnostic_radius) & (arrivals < remaining)
            if overlap.any():
                peer_delay = max(peer_delay, min(float(weights[overlap].sum()), remaining))
        delay += peer_delay
    return delay
