#!/usr/bin/env python3
"""Match a physical obstacle acknowledgement to a genuinely executed reserve."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import numpy as np


def records(file):
    if not file.exists(): return []
    return [json.loads(line) for line in file.read_text().splitlines() if line.strip()]


def reference_distance_change(old,progress,replacement):
    """Geometric cost of a replacement, separate from a causal performance claim."""
    if not np.isfinite(progress) or not 0.<=progress<=old.duration:
        raise ValueError('Reference progress outside interrupted curve')
    points=old.sample_many(np.r_[np.arange(progress,old.duration,.1),old.duration])[0]
    remaining=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    distance=float(np.linalg.norm(np.diff(replacement.path(.1),axis=0),axis=1).sum())
    same_goal=bool(np.linalg.norm(points[-1]-replacement.sample(replacement.duration)[0])<.1)
    return dict(old_remaining_reference_at_injection_m=remaining,newly_authorized_reference_m=distance,
        same_goal=same_goal,reference_distance_change_m=distance-remaining if same_goal else None,
        scope='Replacement reference minus interrupted remaining reference at injection; includes rejoin. This is not a causal comparison against full replanning.')


def switched_execution_window(events, token, region, start):
    """End a reserve's own evidence window at completion or its next cancellation."""
    endpoints = [(e['time'], 'completed' if e['type'] == 'view_observed' else 'cancelled')
                 for e in events if e['time'] >= start and
                 ((e['type'] == 'view_observed' and e.get('token') == token) or
                  (e['type'] == 'lease_cancellation_requested' and
                   e.get('reason') == 'path_or_tracking_invalidated' and
                   e.get('region') == region))]
    return min(endpoints) if endpoints else (None, 'unbounded')


def audit(directory):
    import planning_runtime
    from core.planning.continuous_trajectory import ContinuousTrajectory
    from core.exploration.evidence_audit import ObservationAudit
    root = Path(directory); faults = records(root/'dynamic-backup-fault.jsonl')
    requested = next((p for p in faults if p['type'] == 'obstacle_requested'), None)
    ack = next((p for p in faults if p['type'] == 'obstacle_acknowledged'), None)
    removal = next((p for p in faults if p['type'] == 'obstacle_removal_acknowledged'), None)
    checks = dict(physical_obstacle_requested=requested is not None, gazebo_acknowledged=ack is not None,
                  physical_obstacle_removed=removal is not None)
    details = {}; evidence = None; first_hit = None
    if requested and ack and removal:
        trial = requested['trial']; drone = trial['drone']; start = ack['time']; end = removal['time']
        events = records(root/f'drone_{drone}/events.jsonl')
        executions = records(root/'execution-evidence.jsonl')
        pool = next((p for p in reversed(records(root/f'drone_{drone}/candidates.jsonl'))
                     if p['token'] == trial['token'] and p['time'] <= requested['time']), None)
        reserves = {p['id'] for p in pool['paths'] if p['id'] != pool['active_candidate']} if pool else set()
        active_packet = next((p for p in pool['paths'] if p['id'] == trial['candidate']), None) if pool else None
        active_curve = ContinuousTrajectory.from_dict(active_packet['trajectory']) if active_packet else None
        if active_curve:
            future_points = active_curve.sample_many(np.arange(trial['active_progress'], active_curve.duration, .1))[0]
            blocked = ((np.linalg.norm(future_points[:, :2]-trial['center'], axis=1) < .55)
                       & (future_points[:, 2] > .3) & (future_points[:, 2] < 3.0))
            checks['physical_cylinder_intersects_active_future_curve'] = bool(np.any(blocked))
        else:
            checks['physical_cylinder_intersects_active_future_curve'] = False
        with gzip.open(root/'observations.jsonl.gz', 'rt') as stream:
            for line in stream:
                packet = json.loads(line)
                if evidence is None:
                    evidence = ObservationAudit(packet['shape'], packet['origin'], packet['resolution'], 3)
                evidence.ingest(packet)
                if packet['source'] != drone or not start-.25 <= packet['time'] <= end:
                    continue
                cells = np.asarray(packet.get('measured_indices', []), int).reshape(-1, 3)
                values = np.asarray(packet.get('measured_values', []), int)
                points = np.asarray(packet['origin'])+(cells+.5)*packet['resolution']
                hit = ((values == 1) & (np.linalg.norm(points[:, :2]-trial['center'], axis=1) < .8)
                       & (points[:, 2] > .3) & (points[:, 2] < 3.1))
                if np.any(hit): first_hit = min(first_hit if first_hit is not None else float('inf'), packet['time'])
        invalidation = next((e for e in events if start-.25 <= e['time'] <= end and
            e['type'] == 'lease_cancellation_requested' and e.get('reason') == 'path_or_tracking_invalidated'
            and e.get('region') == trial['region']), None)
        switch = next((e for e in events if start <= e['time'] <= end and e['type'] == 'cached_route_switched'
                       and e['origin']['token'] == trial['token']), None)
        if switch:
            invalidation=next((e for e in events if e['type']=='lease_cancellation_requested'
                and e.get('reason')=='path_or_tracking_invalidated' and e.get('region')==trial['region']
                and abs(e['time']-switch['origin']['time'])<.05),None)
        checks.update(actual_lidar_hit=first_hit is not None,
            actual_route_invalidated=invalidation is not None and first_hit is not None and invalidation['time'] >= first_hit-.25,
            different_preexisting_reserve=switch is not None and switch['candidate'] in reserves and switch['candidate'] != trial['candidate'])
        if switch:
            token = switch['token']
            proposed = next((e for e in events if e['type'] == 'path_proposed' and e.get('token') == token), None)
            selected = next((e for e in events if e['type'] == 'cached_route_selected' and e.get('token') == token), None)
            authorization = next((e for e in events if e['type'] == 'path_committed' and e.get('token') == token), None)
            executed = next((p for p in executions if p['drone'] == drone and p.get('token') == token and
                            p.get('trajectory_time', 0.) > 0 and p.get('reason') == 'tracking_view'), None)
            completion = next((e for e in events if e['type'] == 'view_observed' and e.get('token') == token), None)
            issued = next((p for p in records(root/'command-evidence.jsonl') if p.get('token') == token and p.get('trajectory')), None)
            curve = ContinuousTrajectory.from_dict(issued['trajectory']) if issued else None
            checks['validated_authorized_execution_chain'] = bool(selected and proposed and authorization and executed and curve
                and invalidation and invalidation['time'] <= selected['time'] <= authorization['time']
                and proposed['time'] <= authorization['time'] <= executed.get('time', executed['receipt_time'])+.05
                and issued['epoch'] > trial['epoch'] and issued['token'] != trial['token'])
            window_end, termination = switched_execution_window(
                events, token, trial['region'], authorization['time'] if authorization else switch['time'])
            own_gain = (evidence.window(drone, authorization['time'], window_end)
                        if evidence and authorization and window_end is not None else None)
            checks['positive_actual_team_gain_after_switch'] = bool(own_gain and
                (own_gain['actual_team_new_known'] > 0 or own_gain['actual_team_new_free'] > 0))
            details.update(invalidation=invalidation, selected=selected, proposal=proposed, authorization=authorization,
                           first_actual_execution=executed, completion=completion, switch=switch,
                           actual_curve_limits=curve.limits() if curve else None,
                           switched_trajectory_end=window_end, switched_trajectory_termination=termination,
                           actual_team_gain_during_switched_trajectory=own_gain)
            if executed and first_hit is not None:
                details['sensor_hit_to_authorized_execution_s'] = executed.get('time', executed['receipt_time'])-first_hit
            if authorization and window_end is not None:
                points = []
                with (root/'trajectory.csv').open() as stream:
                    for row in csv.DictReader(stream):
                        if int(row['drone']) == drone and authorization['time'] <= float(row['time']) <= window_end:
                            points.append([float(row[k]) for k in ('x', 'y', 'z')])
                details['actual_recovery_service_distance_m'] = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points)>1 else 0.
                if active_curve and curve:
                    details['route_distance_reference']=reference_distance_change(active_curve,trial['active_progress'],curve)

        details.update(trial=trial, obstacle_start=start, obstacle_end=end, first_actual_lidar_hit=first_hit,
                       cached_candidates_at_injection=sorted(reserves))
    for name, file in [('independent_flight_and_coverage', 'independent-acceptance.json'),
                       ('independent_authorization', 'execution-protocol-audit.json')]:
        checks[name] = file is not None and (root/file).exists() and json.loads((root/file).read_text()).get('passed') is True
    summary = json.loads((root/'summary.json').read_text())
    checks['reached_95'] = summary['status'] == 'COMPLETE'
    result = dict(passed=all(checks.values()), checks=checks, details=details,
                  note='This is one externally targeted physical-obstacle case. It is separate from the preregistered efficiency and stock recovery groups.')
    (root/'dynamic-backup-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('directory'); args = parser.parse_args()
    result = audit(args.directory); print(json.dumps(result, indent=2)); raise SystemExit(0 if result['passed'] else 1)
