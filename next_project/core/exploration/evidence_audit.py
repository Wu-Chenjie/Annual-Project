"""Offline earliest-real-observation attribution, independent of planner promises."""
import numpy as np


class ObservationAudit:
    def __init__(self, shape, origin, resolution, fleet_size=3):
        self.shape = tuple(shape); self.origin = np.asarray(origin); self.resolution = resolution
        n = int(np.prod(shape)); self.seen_packets = set(); self.frames = 0
        self.first = np.full(n, np.inf); self.owner = np.full(n, fleet_size, int)
        self.first_free = np.full(n, np.inf); self.free_owner = np.full(n, fleet_size, int)
        self.local_first = np.full((fleet_size, n), np.inf)
        self.local_free = np.full((fleet_size, n), np.inf)

    def ingest(self, packet):
        source = int(packet['source']); stamp = float(packet['time'])
        if not 0 <= source < len(self.local_first) or not np.isfinite(stamp):
            raise ValueError('Invalid source or observation time')
        identity = (source, packet.get('sensor_session'), int(packet['sequence']))
        if identity in self.seen_packets: return False
        if (tuple(packet['shape']) != self.shape or packet['resolution'] != self.resolution or
                not np.array_equal(packet['origin'], self.origin)):
            raise ValueError('Observation evidence grid mismatch')
        # A bootstrap retransmits old map cells. Only this frame's measured
        # subset counts as evidence; snapshots never create new sensor times.
        indices = np.asarray(packet['measured_indices'], int).reshape(-1, len(self.shape))
        values = np.asarray(packet['measured_values'], int)
        if len(values) != len(indices) or not np.isin(values, [0, 1]).all() or np.any(indices < 0) or np.any(indices >= self.shape):
            raise ValueError('Invalid measured-cell evidence')
        ids = np.ravel_multi_index(tuple(indices.T), self.shape) if len(indices) else np.empty(0, int)
        free = np.unique(ids[values == 0]); ids = np.unique(ids)
        self.local_first[source, ids] = np.minimum(stamp, self.local_first[source, ids])
        self.local_free[source, free] = np.minimum(stamp, self.local_free[source, free])
        for cells, times, owners in ((ids, self.first, self.owner), (free, self.first_free, self.free_owner)):
            earlier = (stamp < times[cells]) | ((stamp == times[cells]) & (source < owners[cells]))
            selected = cells[earlier]; times[selected] = stamp; owners[selected] = source
        self.seen_packets.add(identity); self.frames += 1
        return True

    def window(self, source, start, end):
        if not np.isfinite([start, end]).all() or end < start: raise ValueError('Invalid service window')
        local = (self.local_first[source] > start) & (self.local_first[source] <= end)
        team = (self.owner == source) & (self.first > start) & (self.first <= end)
        local_free = (self.local_free[source] > start) & (self.local_free[source] <= end)
        team_free = (self.free_owner == source) & (self.first_free > start) & (self.first_free <= end)
        return dict(local_new_known=int(local.sum()), actual_team_new_known=int(team.sum()),
                    local_new_free=int(local_free.sum()), actual_team_new_free=int(team_free.sum()),
                    duplicate_local_new_known=int(np.count_nonzero(local & ~team)))
