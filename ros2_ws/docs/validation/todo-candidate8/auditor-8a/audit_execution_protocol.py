#!/usr/bin/env python3
"""Independently join proposals, dual authorizations and actual executor tokens."""
import argparse
import hashlib
import json
from pathlib import Path

import planning_runtime
from core.planning.continuous_trajectory import ContinuousTrajectory
from core.planning.handoff import validate_handoff


def records(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():yield json.loads(line)


def check(events,states,executions,fleet_size,end_time,commands=()):
    events=sorted(events,key=lambda e:(e['time'],e['drone']));members=set(range(fleet_size))
    intents={};errors=[];proposals={};authorized={};intervals=[];handoffs=[];switches=[];cancelled={};legacy_labels=[]
    def require(condition,message):
        if not condition:errors.append(message)
    for state in states:
        for intent in [state.get('intent'),state.get('pending_intent'),*state.get('retiring_intents',[])]:
            if not intent:continue
            intents.setdefault(intent['token'],dict(intent=intent,drone=state['drone']))
    captured=0
    for command in commands:
        if not command.get('trajectory') or not command.get('token'):continue
        captured+=1;token=command['token'];digest=hashlib.sha256(json.dumps(command['trajectory'],sort_keys=True).encode()).hexdigest()
        record=intents.setdefault(token,dict(intent=command,drone=command['drone']))
        require(record['drone']==command['drone'],f'Command source changed under token {token}')
        for key in ('epoch','region','yaw','candidate_id','handoff'):
            if key in record['intent'] and key in command:
                require(record['intent'][key]==command[key],f'Command {key} changed under token {token}')
        if 'digest' in record:require(record['digest']==digest,f'Curve changed under token {token}')
        record['digest']=digest;record['intent']=dict(record['intent'],trajectory=command['trajectory'])
    modern_cache_metadata=(any('candidate_id' in record['intent'] for record in intents.values()) or
                           any(e['type'].startswith('handoff_') for e in events))
    for event in events:
        kind=event['type'];token=event.get('token');drone=event['drone']
        if kind in ('path_proposed','handoff_proposed'):
            require(token not in proposals,f'Reused proposal {token}');proposals[token]=event
        elif kind in ('path_committed','handoff_authorized'):
            proposal=proposals.get(token);record=intents.get(token)
            require(proposal is not None,f'Authorization has no proposal {token}')
            require(record is not None and 'digest' in record,f'Authorization has no captured curve {token}')
            if proposal is None or record is None or 'digest' not in record:continue
            intent=record['intent'];quorum=set(event['quorum']);voters=set(intent.get('voters',[]))
            require(record['drone']==drone and proposal['drone']==drone,f'Wrong token source {token}')
            require(quorum==voters,f'ACK mismatch {token}')
            require(intent.get('contingency') or quorum==members-{drone},f'Incomplete fleet ACK {token}')
            require(proposal['time']<=event['time'],f'Authorization predates proposal {token}')
            require(token not in authorized,f'Reused authorization {token}')
            try:
                new=ContinuousTrajectory.from_dict(intent['trajectory'])
                if kind=='handoff_authorized':
                    boundary=intent['handoff'];old_token=boundary['from_token']
                    require(old_token in authorized,f'Old lease absent during handoff {token}')
                    old=ContinuousTrajectory.from_dict(intents[old_token]['intent']['trajectory'])
                    continuity=validate_handoff(old,new,boundary['trajectory_time'])
                    handoffs.append(dict(token=token,old_token=old_token,authorized_at=event['time'],continuity=continuity))
            except (ValueError,KeyError,TypeError) as exc:errors.append(f'Invalid authorized curve {token}: {exc}')
            authorized[token]=dict(token=token,drone=drone,region=intent['region'],start=event['time'],end=end_time)
        elif kind=='reservation_retired' and token in authorized:
            authorized[token]['end']=event['time'];intervals.append(authorized.pop(token))
        elif kind=='handoff_cancelled':
            cancelled[token]=event['time']
            if token in authorized:
                authorized[token]['end']=event['time'];intervals.append(authorized.pop(token))
        elif kind=='view_observed' and token in authorized and not any(h['old_token']==token for h in handoffs):
            # Ordinary completion removes the current intent in the same callback.
            authorized[token]['end']=event['time'];intervals.append(authorized.pop(token))
        elif kind=='incarnation_ready':
            for old_token,value in list(authorized.items()):
                if value['drone']==drone and event.get('incarnation') not in old_token:
                    value['end']=event['time'];intervals.append(authorized.pop(old_token))
        elif kind=='cached_route_switched':
            if not modern_cache_metadata and not any(k in event for k in ('token','candidate','origin')):
                # The fixed early baseline logged a region-only cache label
                # before a new commit. It proves no actual reserve execution.
                # Authorizations and all executor tokens are still checked.
                legacy_labels.append(event)
            else:switches.append(event)
    intervals.extend(authorized.values())
    for i,a in enumerate(intervals):
        for b in intervals[i+1:]:
            require(not (a['drone']!=b['drone'] and a['region']==b['region'] and min(a['end'],b['end'])-max(a['start'],b['start'])>.05),
                    f'Overlapping regional leases {a["token"]}, {b["token"]}')
    actual={}
    for packet in executions:
        token=packet.get('token')
        if token and packet.get('trajectory_time',0.)>0 and packet.get('reason') in ('tracking_view','observation_dwell','view_observed'):
            actual.setdefault(token,packet)
            leases=[lease for lease in intervals if lease['token']==token and lease['drone']==packet.get('drone',lease['drone'])]
            require(bool(leases),f'Executed token has no recorded authorization {token}')
            stamp=packet.get('time',packet.get('receipt_time'))
            if stamp is not None:
                require(any(lease['start']-.25<=stamp<=lease['end']+.25 for lease in leases),
                        f'Executed outside authorized lease interval {token}')
    for handoff in handoffs:
        packet=actual.get(handoff['token'])
        if handoff['token'] in cancelled and packet is None:
            handoff['cancelled_at']=cancelled[handoff['token']];continue
        require(packet is not None,f'Authorized handoff not consumed {handoff["token"]}')
        if packet:
            require(packet.get('handoff_from_token')==handoff['old_token'],f'Wrong executed predecessor {handoff["token"]}')
            require(packet.get('handoff_time',-1.)>=handoff['authorized_at']-.05,f'Execution before authorization {handoff["token"]}')
            handoff['executed_at']=packet.get('handoff_time')
    for event in switches:
        if (not all(k in event for k in ('token','origin','candidate')) or not isinstance(event['origin'],dict)
                or not all(k in event['origin'] for k in ('time','candidate'))):
            require(False,'Incomplete modern cached-route metadata');continue
        token=event['token'];origin=event['origin'];proposal=proposals.get(token)
        selected=[e for e in events if e['drone']==event['drone'] and e['type']=='cached_route_selected' and
                  origin['time']<=e['time']<=event['time'] and e.get('candidate')==event['candidate']]
        require(bool(selected) and proposal is not None and origin['time']<=proposal['time']<=event['time'],f'Incomplete cached-route chain {token}')
        require(origin.get('candidate') is not None and origin['candidate']!=event['candidate'],f'Cached switch reused active candidate {token}')
        require(intents.get(token,{}).get('intent',{}).get('candidate_id')==event['candidate'],f'Executed cached candidate mismatch {token}')
        require(token in actual,f'Cached-route authorization never executed {token}')
    return dict(passed=not errors and captured>0,status='INCOMPLETE_OBSERVABILITY' if not captured else 'PASS' if not errors else 'FAIL',
                errors=errors,proposals=len(proposals),captured_curve_commands=captured,
                lease_intervals=len(intervals),handoffs=handoffs,cached_route_chains=switches,
                legacy_cache_labels_without_execution_provenance=legacy_labels,
                note='Telemetry verifies captured curve immutability, analytic limits, C2 boundaries, logged ACK sets, regional exclusion and actual executor tokens. It does not certify unrecorded packets or replace collision/true-flight acceptance.')


def audit(directory):
    root=Path(directory);s=json.loads((root/'summary.json').read_text())
    result=check([e for file in root.glob('drone_*/events.jsonl') for e in records(file)],records(root/'peer_states.jsonl'),
                 records(root/'execution-evidence.jsonl'),s['fleet_size'],s['simulation_time'],
                 records(root/'command-evidence.jsonl') if (root/'command-evidence.jsonl').exists() else ())
    (root/'execution-protocol-audit.json').write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');args=p.parse_args();result=audit(args.directory)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
