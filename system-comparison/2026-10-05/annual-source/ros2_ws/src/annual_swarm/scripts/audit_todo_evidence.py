#!/usr/bin/env python3
"""Reconstruct per-window team novelty and residence from raw run evidence."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import numpy as np
import planning_runtime
from core.exploration.evidence_audit import ObservationAudit
from core.exploration.timing_evidence import parent_geometry_timings
from core.exploration.service_windows import recover_service_windows
from core.exploration.execution_identity import bind_execution_identities


def audit(directory):
    directory = Path(directory); summary = json.loads((directory/'summary.json').read_text())
    evidence = None
    with gzip.open(directory/'observations.jsonl.gz', 'rt') as stream:
        for line in stream:
            packet = json.loads(line)
            if evidence is None:
                evidence = ObservationAudit(packet['shape'], packet['origin'], packet['resolution'], summary['fleet_size'])
            evidence.ingest(packet)
    if evidence is None: raise ValueError('No actual observation evidence')
    events = []; states = []
    for file in sorted(directory.glob('drone_*/events.jsonl')):
        events += [json.loads(line) for line in file.read_text().splitlines()]
    states=[json.loads(line) for line in (directory/'peer_states.jsonl').read_text().splitlines()]
    executions=directory/'execution-evidence.jsonl'
    execution_records=[json.loads(line) for line in executions.read_text().splitlines()] if executions.exists() else []
    if not execution_records:raise ValueError('Actual execution stream required for service-window accounting')
    commands=[json.loads(line) for line in (directory/'command-evidence.jsonl').read_text().splitlines()] if (directory/'command-evidence.jsonl').exists() else []
    identities=bind_execution_identities(execution_records,commands,
        modern_required=any('candidate_id' in command for command in commands))
    if identities['errors']:raise ValueError('Invalid executor identity: '+str(identities['errors']))
    execution_records=identities['packets']
    metadata={}
    for state in states:
        for intent in [state.get('intent'),state.get('pending_intent')]+state.get('retiring_intents',[]):
            if intent and intent.get('token'):metadata[(state['drone'],intent['token'])]=intent
    task_start=summary.get('start_time') or 0.;task_end=summary.get('finish_time') or summary['simulation_time']
    recovered=recover_service_windows(events,execution_records,task_start,task_end)
    candidate_purposes={}
    for file in sorted(directory.glob('drone_*/candidates.jsonl')):
        for line in file.open():
            packet=json.loads(line);paths=packet.get('paths',[])
            purpose=packet.get('purpose')
            if purpose is None and paths:
                planner=paths[0].get('planner')
                purpose='transit_reobserve' if planner=='mrdtg_transit' else 'safety_recovery' if packet.get('region',0)<0 else 'explore'
            if purpose:
                candidate_purposes[(packet['drone'],str(packet['epoch']))]=(purpose,packet['region'])
    windows = []; assigned = 0
    tracks = {}
    with (directory/'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            tracks.setdefault(int(row['drone']), []).append([float(row[k]) for k in ('time','x','y','z')])
    cumulative = {}
    for i, data in tracks.items():
        points = np.array(data); cumulative[i] = (points[:,0], np.r_[0.,np.linalg.norm(np.diff(points[:,1:],axis=0),axis=1).cumsum()])
    for window in recovered['windows']:
        end=window['receipt'] or {};commit=window['commit'];intent=metadata.get((window['drone'],window['token']),{})
        start=window['start'];finished=window['end'];region=end.get('region',intent.get('region',window['region']))
        purpose=end.get('purpose',intent.get('purpose'))
        if purpose is None:
            candidate=candidate_purposes.get((window['drone'],str(window['token']).rsplit(':',1)[-1]))
            if candidate:
                purpose=candidate[0];region=candidate[1] if region is None else region
            elif region is not None and region<0:purpose='safety_recovery'
            else:raise ValueError('Missing actual service purpose for '+window['token'])
        row = dict(drone=window['drone'], token=window['token'], region=region, start=start, end=finished,
                   purpose=purpose, duration_s=finished-start,termination=window['termination'],
                   online_regional_team_new_estimate=end.get('team_new_cells'))
        row.update(evidence.window(window['drone'], start, finished))
        times, distances = cumulative[window['drone']]
        row['distance_m'] = float(np.interp(finished,times,distances)-np.interp(start,times,distances))
        row['low_team_yield'] = row['actual_team_new_known'] < 30
        row['zero_team_yield'] = row['actual_team_new_known'] == 0
        windows.append(row); assigned += row['actual_team_new_known']
    for drone in range(summary['fleet_size']):
        ordered=sorted((w for w in windows if w['drone']==drone),key=lambda w:w['start'])
        if any(a['end']>b['start']+1e-6 for a,b in zip(ordered,ordered[1:])):
            raise ValueError('Overlapping actual service windows')
    if assigned>int(np.isfinite(evidence.first).sum()):
        raise ValueError('Service windows duplicated team-new observations')
    residence = {}; previous = {}; close = {}; near_samples = []
    for state in states:
        i = state['drone']; t = state['time']
        old = previous.get(i)
        if old and 0 <= t-old['time'] <= 1.5:
            execution = old.get('execution', {}); reason = execution.get('reason', 'unrecorded_baseline')
            residence[reason] = residence.get(reason, 0.)+t-old['time']
            if i == 0:
                for a in range(summary['fleet_size']):
                  for j in range(a+1, summary['fleet_size']):
                    one = old if a==0 else previous.get(a); peer = previous.get(j)
                    if one and peer and abs(old['time']-peer['time']) < .6 and abs(old['time']-one['time']) < .6:
                        d = float(np.linalg.norm(np.asarray(one['position'])-peer['position']))
                        if d < 2.:
                            key = f'{a}:{j}'; close[key] = close.get(key, 0.)+t-old['time']
                            near_samples.append(dict(time=old['time'],pair=key,distance_m=d,
                                duration_s=t-old['time'],region_a=(one.get('intent') or {}).get('region'),
                                region_b=(peer.get('intent') or {}).get('region'),
                                purpose_a=(one.get('intent') or {}).get('purpose','baseline_unspecified'),
                                purpose_b=(peer.get('intent') or {}).get('purpose','baseline_unspecified'),
                                reason_a=one.get('execution',{}).get('reason'),
                                reason_b=peer.get('execution',{}).get('reason'),
                                token_a=(one.get('intent') or {}).get('token'),token_b=(peer.get('intent') or {}).get('token')))
        previous[i] = state
    executions = directory/'execution-evidence.jsonl'
    phases={i:{} for i in range(summary['fleet_size'])}
    if executions.exists():
        residence = {}; previous = {}
        timeline={i:[] for i in range(summary['fleet_size'])}
        for state in states:timeline[state['drone']].append(state)
        clocks={i:np.array([s['time'] for s in rows]) for i,rows in timeline.items()}
        for line in executions.read_text().splitlines():
            packet = json.loads(line); i=packet['drone']; old=previous.get(i)
            if old and 0 <= packet['receipt_time']-old['receipt_time'] <= .6:
                dt = max(0., min(packet['receipt_time'], summary.get('finish_time') or np.inf)-max(old['receipt_time'],summary.get('start_time') or 0.))
                residence[old['reason']]=residence.get(old['reason'],0.)+dt
                j=int(np.searchsorted(clocks[i],old['receipt_time'],side='right'))-1
                state=timeline[i][j] if j>=0 else {}
                phase=old['reason']; intent=state.get('intent') or {}
                if phase=='tracking_view':
                    phase={'transit_reobserve':'navigation_reobserve','safety_recovery':'safety_recovery'}.get(intent.get('purpose'),'execution')
                elif phase=='initial_yaw_scan':phase='observation'
                elif phase=='idle':
                    if intent and not intent.get('committed'):phase='reservation_wait'
                    elif state.get('fusion',{}).get('worker_running') or state.get('fusion',{}).get('fit_queued'):phase='planning_wait'
                    elif state.get('motion_blocked'):phase='safety_hold'
                    elif not state.get('available',True):phase='experiment_pause'
                    else:phase='allocation_wait'
                phases[i][phase]=phases[i].get(phase,0.)+dt
            previous[i]=packet
    planning = [e for e in events if e['type'] == 'planning_result']
    times = [e['total_wall_s'] for e in planning]
    if not times:
        times=list({(p['drone'],p.get('session'),p['planner_wall_s']):p['planner_wall_s'] for p in states if p.get('planner_wall_s',0)>0}.values())
    compute_times=[e['compute_wall_s'] for e in planning] if planning else times
    def quantiles(values):
        return {f'p{x}':float(np.quantile(values,x/100)) for x in (50,90,95)} if values else None
    measured_requests=[e['total_wall_s'] for e in events if e['type'] in ('planning_result','planning_timeout','planning_failed') and 'total_wall_s' in e]
    stages=sorted({k for e in planning for k in (e.get('stage_wall_s') or {})})
    cover = json.loads((directory/'coverage.json').read_text()); start = summary.get('start_time')
    thresholds = {f't{int(x*100)}': next((t-start for t, c in cover if start is not None and c >= x and t >= start), None)
                  for x in (.5, .8, .9, .95)}
    moving = [e for e in events if e['type'] == 'handoff_consumed']
    result = dict(schema='annual.todo-audit/2', status=summary['status'], thresholds=thresholds,
        tail_90_95_s=thresholds['t95']-thresholds['t90'] if thresholds['t95'] is not None else None,
        service_window_definition=recovered['definition'],
        service_window_terminations={kind:sum(w['termination']==kind for w in windows) for kind in sorted({w['termination'] for w in windows})},
        never_executed_authorizations=len(recovered['never_executed_authorizations']),
        legacy_executor_epoch_bindings=identities['legacy_bound_packets'],
        post_cancel_static_hold_packets=identities['post_cancel_static_hold_packets'],
        actual_tracking_packets=sum(p.get('reason')=='tracking_view' and p.get('trajectory_time',0)>0 for p in execution_records),
        raw_frames=evidence.frames, observed_known_cells=int(np.isfinite(evidence.first).sum()),
        observed_free_cells=int(np.isfinite(evidence.first_free).sum()), service_windows=len(windows),
        actual_team_new_known_in_windows=assigned,
        team_new_known_outside_service_windows=int(np.isfinite(evidence.first).sum())-assigned,
        exploration_low_team_yield=sum(w['low_team_yield'] for w in windows if w['purpose']=='explore'),
        exploration_zero_team_yield=sum(w['zero_team_yield'] for w in windows if w['purpose']=='explore'),
        exploration_low_yield_duration_s=sum(w['duration_s'] for w in windows if w['purpose']=='explore' and w['low_team_yield']),
        exploration_total_duration_s=sum(w['duration_s'] for w in windows if w['purpose']=='explore'),
        exploration_total_distance_m=sum(w['distance_m'] for w in windows if w['purpose']=='explore'),
        exploration_low_yield_distance_m=sum(w['distance_m'] for w in windows if w['purpose']=='explore' and w['low_team_yield']),
        purpose_counts={p:sum(w['purpose']==p for w in windows) for p in ('explore','transit_reobserve','safety_recovery')},
        residence_s=residence, close_radius_m=2., close_pair_duration_s=close,
        residence_by_uav_s=phases,
        planning_wait_fleet_s=sum(row.get('planning_wait',0.) for row in phases.values()) if executions.exists() else None,
        planning_wall_median_s=float(np.median(times)) if times else None,
        planning_wall_p95_s=float(np.quantile(times,.95)) if times else None,
        planning_compute_wall_median_s=float(np.median(compute_times)) if compute_times else None,
        planning_request_counts={kind:sum(e['type']==kind for e in events) for kind in ('planning_submitted','planning_result','planning_timeout','planning_failed')},
        planning_all_measured_requests_wall_s=quantiles(measured_requests),
        planning_queue_wall_s=quantiles([e['dispatch_queue_wall_s'] for e in planning if 'dispatch_queue_wall_s' in e]),
        planning_delivery_wall_s=quantiles([e['result_delivery_wall_s'] for e in planning if 'result_delivery_wall_s' in e]),
        planning_stage_wall_s={k:quantiles([e['stage_wall_s'][k] for e in planning if k in (e.get('stage_wall_s') or {})]) for k in stages},
        parent_geometry=parent_geometry_timings(states, summary.get('start_time') or 0.,
                                               summary.get('finish_time') or summary['simulation_time']),
        parent_proposal=parent_geometry_timings(states, summary.get('start_time') or 0.,
                                               summary.get('finish_time') or summary['simulation_time'],'parent_proposal'),
        rejected_planning_results=sum(bool(e.get('rejection')) for e in planning),
        planning_time_scope='submitted_to_result_receipt' if planning else 'worker_compute_only_unique_samples',
        planning_timeout_count=sum(e['type']=='planning_timeout' for e in events), moving_handoffs=len(moving),
        moving_handoff_max_boundary_error=max((max(e.get('continuity', {}).values(), default=0.) for e in moving), default=None),
        distances_m=summary['distances_m'], coverage=summary['coverage'], contacts=summary['contacts_after_takeoff'],
        completed_window_duplicate_ratio=dict(numerator=sum(w['duplicate_local_new_known'] for w in windows),
            denominator=sum(w['local_new_known'] for w in windows),scope='sum of actual local-new cells in completed, cancelled, restarted and task-end service windows'),
        limitations=['Online regional estimates and offline full service-window gains have different spatial scopes.',
                      'Close duration is telemetry sampled; missing intervals are excluded.',
                      'First-free evidence does not replace final truth-free coverage acceptance.'])
    (directory/'todo-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    with (directory/'service-windows.csv').open('w') as stream:
        if windows:
            writer=csv.DictWriter(stream, fieldnames=list(windows[0]));writer.writeheader();writer.writerows(windows)
    with (directory/'near-pair-samples.csv').open('w') as stream:
        if near_samples:
            writer=csv.DictWriter(stream,fieldnames=list(near_samples[0]));writer.writeheader();writer.writerows(near_samples)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    print(json.dumps(audit(args.directory), indent=2))
