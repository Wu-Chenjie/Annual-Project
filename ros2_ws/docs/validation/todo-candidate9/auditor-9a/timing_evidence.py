"""Windowed cumulative timing counters, including process reincarnations."""


def parent_geometry_timings(states, start, end, prefix='parent_geometry'):
    records = {}
    wall_key=prefix+'_wall_s';count_key=prefix+'_count'
    count_name='rebuilds' if prefix=='parent_geometry' else 'calls'
    observed_drones = {p['drone'] for p in states if start < p['time'] <= end}
    for packet in states:
        if packet['time'] > end:
            continue
        fields = packet.get('fusion', {})
        if wall_key not in fields:
            continue
        records.setdefault((packet['drone'], packet['session']), []).append(packet)
    if not records:
        return dict(status='UNAVAILABLE', wall_s=None, **{count_name:None},
                    note='Legacy telemetry has no '+prefix+' counter; missing is not zero.')
    sessions = []
    for (drone, session), packets in sorted(records.items()):
        packets.sort(key=lambda p: p['time'])
        before = {wall_key:0.,count_key:0}
        last = before
        for packet in packets:
            current = packet['fusion']
            if any(current[key] < last[key] for key in (wall_key,count_key)):
                raise ValueError('Parent timing counters decreased within one process session')
            if packet['time'] <= start:
                before = {wall_key:current[wall_key],count_key:current[count_key]}
            last = current
        sessions.append(dict(drone=drone, session=session,
            wall_s=last[wall_key]-before[wall_key],
            **{count_name:last[count_key]-before[count_key]}))
    complete = observed_drones <= {s['drone'] for s in sessions}
    return dict(status='RECORDED' if complete else 'PARTIAL',
                wall_s=sum(s['wall_s'] for s in sessions) if complete else None,
                **{count_name:sum(s[count_name] for s in sessions) if complete else None}, sessions=sessions,
                note='Cumulative parent callback counters differenced over the task window. '
                     'Worker map timing is separate; parallel durations are not end-to-end latency.')
