#!/usr/bin/env python3
"""Separate native planning, perception and neighbor-safety recovery evidence."""
import argparse
import gzip
import json
from pathlib import Path


def records(path):
    if not path.exists():return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def audit(root):
    root=Path(root);events=[e for p in root.glob('drone_*/events.jsonl') for e in records(p)]
    executions=records(root/'execution-evidence.jsonl');states=records(root/'peer_states.jsonl')
    accepted=json.loads((root/'independent-acceptance.json').read_text())
    protocol=json.loads((root/'execution-protocol-audit.json').read_text());summary=json.loads((root/'summary.json').read_text())
    checks={};cases=[]
    injected=records(root/'native-fault-events.jsonl')
    for item in (e for e in injected if e['type']=='fault_injected'):
        case=item['case'];drone=item['drone'];stamp=item['time']
        ended=next(e for e in injected if e['type']=='fault_window_ended' and e['case']==case)
        same=[e for e in events if e['drone']==drone and e['time']>=stamp]
        times=sorted(s['time'] for s in states if s['drone']==drone and stamp-.4<=s['time']<=max(ended['time'],stamp+5.)+.4)
        gap=max((b-a for a,b in zip(times,times[1:])),default=float('inf'))
        details=dict(case=case,drone=drone,injected_at=stamp,ended_at=ended['time'],maximum_heartbeat_gap_s=gap)
        checks[case+'_heartbeat_responsive']=len(times)>3 and gap<1.2
        if case in ('planning_deadline','planning_worker_crash'):
            kind='planning_timeout' if case=='planning_deadline' else 'planning_failed'
            failure=next((e for e in same if e['type']==kind and e.get('request_id')==item['request_id']),None)
            checks[case+'_matching_request_failed']=failure is not None
            checks[case+'_recomputed_after_failure']=failure is not None and any(e['type']=='planning_result' and e['time']>failure['time'] for e in same)
            checks[case+'_genuine_observations_after_failure']=any(e['type']=='view_observed' and e.get('new_cells',0)>0 and e['time']>ended['time'] for e in same)
            if case=='planning_deadline':checks['planning_deadline_within_budget']=failure is not None and failure['total_wall_s']<=12.
            details['failure']=failure;details['original_worker_alive_after_window']=ended['original_process_alive']
        else:
            checks['perception_loss_withdraws_exploration']=any(e['type']=='lease_cancellation_requested' and e.get('reason')=='observation_timeout' for e in same)
            checks['perception_mapper_resumed']=ended['resumed']
            checks['perception_restores_positive_service']=any(e['type']=='view_observed' and e.get('new_cells',0)>0 and e['time']>ended['time'] for e in same)
        cases.append(details)
    relay=records(root/'safety-input-fault.jsonl')
    if relay:
        start=next((e for e in relay if e['type']=='safety_input_outage_started'),None)
        end=next((e for e in relay if e['type']=='safety_input_outage_ended'),None)
        checks['neighbor_safety_input_was_suppressed']=bool(start and end and end['suppressed']>0)
        if start and end:
            held=[p for p in executions if p['drone']==0 and start['time']+.65<=p['receipt_time']<end['time']]
            checks['neighbor_safety_watchdog_holds']=len(held)>=4 and all(p['reason']=='odometry_timeout' for p in held)
            progress={}
            for p in held:
                if p.get('token'):progress.setdefault(p['token'],[]).append(p['trajectory_time'])
            checks['neighbor_safety_no_curve_advance']=bool(progress) and all(max(v)-min(v)<1e-8 for v in progress.values())
            checks['neighbor_safety_resumes_motion']=any(p['drone']==0 and p['receipt_time']>end['time'] and p['reason']=='tracking_view' for p in executions)
            checks['neighbor_safety_restores_positive_service']=any(e['drone']==0 and e['time']>end['time'] and e['type']=='view_observed' and e.get('new_cells',0)>0 for e in events)
            cases.append(dict(case='neighbor_safety_input_loss',start=start,end=end,
                              note='Only executor0 drone2 safety odometry was suppressed. Own control/EKF and planning peer topics were unchanged.'))
    if injected:checks['all_three_native_signal_cases_executed']=len(cases)==3
    checks.update(independent_flight_and_coverage=accepted['passed'],independent_authorization=protocol['passed'],reached_95=summary['status']=='COMPLETE')
    result=dict(passed=all(checks.values()),checks=checks,cases=cases,
                note='These bounded inputs establish only the recorded cases. Planning, perception and neighbor safety are separate failure boundaries.')
    (root/'native-fault-audit.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    result=audit(args.directory);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
