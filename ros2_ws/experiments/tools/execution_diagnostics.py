#!/usr/bin/env python3
"""Reconstruct execution phases, measured low-speed time and planning overlap."""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def records(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip(): yield json.loads(line)


def merged(intervals):
    output=[]
    for start,end in sorted(intervals):
        if end<=start:continue
        if output and start<=output[-1][1]:output[-1]=(output[-1][0],max(end,output[-1][1]))
        else:output.append((start,end))
    return output


def overlap(start,end,intervals):
    return sum(max(0.,min(end,b)-max(start,a)) for a,b in intervals)


def phase(execution,peer):
    reason=execution['reason']
    if reason=='tracking_view':
        return {'transit_reobserve':'navigation_reobserve','safety_recovery':'safety_recovery'}.get(peer.get('purpose'),'execution')
    if reason=='initial_yaw_scan':return 'observation'
    if reason!='idle':return reason
    if peer.get('uncommitted'):return 'reservation_wait'
    if peer.get('worker'):return 'planning_wait'
    if peer.get('motion_blocked'):return 'safety_hold'
    if peer.get('available') is False:return 'experiment_pause'
    return 'allocation_wait'


def planning_intervals(events,start,end):
    starts={};intervals=defaultdict(list);counts=Counter();unmatched=0
    for e in sorted(events,key=lambda e:e['time']):
        key=(e['drone'],e.get('incarnation'),e.get('request_id'))
        if e['type']=='planning_submitted' and e.get('request_id'):
            starts[key]=e['time']
        elif e['type'] in ('planning_result','planning_failed','planning_timeout'):
            if start<=e['time']<=end:counts[e['type']]+=1
            begin=starts.pop(key,None) if e.get('request_id') else None
            if begin is None:
                if start<=e['time']<=end:unmatched+=1
            else:intervals[e['drone']].append((max(start,begin),min(end,e['time'])))
    for (drone,_,_),begin in starts.items():
        if begin<end:intervals[drone].append((max(start,begin),end))
    return {i:merged(value) for i,value in intervals.items()},dict(counts),unmatched


def quantiles(values):
    return dict(samples=len(values),p50=float(np.quantile(values,.5)),p95=float(np.quantile(values,.95)),maximum=max(values)) if values else None


def diagnose(directory,output,low_speed=.1):
    directory,output=Path(directory),Path(output)
    output.mkdir(parents=True,exist_ok=True)
    summary=json.loads((directory/'summary.json').read_text())
    start=summary['start_time'];end=summary.get('finish_time') or summary['simulation_time']
    events=[e for p in directory.glob('drone_*/events.jsonl') for e in records(p)]
    events.sort(key=lambda e:(e['time'],e['drone']))
    requests,request_counts,unmatched=planning_intervals(events,start,end)
    # Keep only diagnostic fields; private graph/trajectory snapshots are large.
    peers=defaultdict(list)
    for p in records(directory/'peer_states.jsonl'):
        intent=p.get('intent') or {};fusion=p.get('fusion') or {}
        peers[p['drone']].append(dict(time=p['time'],purpose=intent.get('purpose'),
            uncommitted=bool(intent and not intent.get('committed')),worker=bool(fusion.get('worker_running') or fusion.get('fit_queued')),
            motion_blocked=p.get('motion_blocked'),available=p.get('available')))
    for value in peers.values():value.sort(key=lambda p:p['time'])
    peer_clocks={i:[p['time'] for p in value] for i,value in peers.items()}
    poses=defaultdict(list)
    with (directory/'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            poses[int(row['drone'])].append([float(row[k]) for k in ('time','x','y','z')])
    motion={}
    for i,value in poses.items():
        a=np.asarray(value);dt=np.diff(a[:,0]);ok=(dt>0)&(dt<=.6)
        motion[i]=(a[:-1,0][ok],a[1:,0][ok],np.linalg.norm(np.diff(a[:,1:],axis=0),axis=1)[ok]/dt[ok])
    previous={};timeline=[];residence=defaultdict(float);low=defaultdict(float)
    by_uav=defaultdict(lambda:defaultdict(float));motion_covered=0.;total_low=0.;total_high=0.;planning_overlap=0.
    planning_during_motion=0.;planning_during_low=0.;max_speed=0.
    for packet in records(directory/'execution-evidence.jsonl'):
        i=packet['drone'];stamp=packet.get('time',packet['receipt_time']);old=previous.get(i);previous[i]=packet
        if old is None:continue
        old_time=old.get('time',old['receipt_time']);a=max(start,old_time);b=min(end,stamp)
        if b<=a or not 0<stamp-old_time<=.6:continue
        index=bisect.bisect_right(peer_clocks.get(i,[]),a)-1
        peer=peers[i][index] if index>=0 and a-peers[i][index]['time']<=1.5 else {}
        label=phase(old,peer)
        residence[label]+=b-a;by_uav[i][label]+=b-a
        x,y,speed=motion[i];left=np.searchsorted(y,a,side='right');right=np.searchsorted(x,b,side='left')
        low_time=high_time=covered=distance=0.
        for j in range(left,right):
            p=max(a,float(x[j]));q=min(b,float(y[j]));dt=q-p
            if dt<=0:continue
            covered+=dt;distance+=float(speed[j])*dt;max_speed=max(max_speed,float(speed[j]))
            concurrent=overlap(p,q,requests.get(i,[]))
            if speed[j]<low_speed:low_time+=dt;planning_during_low+=concurrent
            else:high_time+=dt;planning_during_motion+=concurrent
        concurrent=overlap(a,b,requests.get(i,[]));planning_overlap+=concurrent
        low[label]+=low_time;total_low+=low_time;total_high+=high_time;motion_covered+=covered
        timeline.append(dict(drone=i,start=a,end=b,phase=label,reason=old['reason'],token=old.get('token'),
                             epoch=old.get('epoch'),low_speed_s=low_time,moving_s=high_time,
                             motion_samples_covered_s=covered,mean_sampled_speed_mps=distance/covered if covered else None,
                             planning_request_overlap_s=concurrent))
    with (output/'execution-timeline.csv').open('w') as stream:
        fields=list(timeline[0]) if timeline else ['drone','start','end','phase']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(timeline)
    relevant=[e for e in events if start<=e['time']<=end]
    counts=Counter(e['type'] for e in relevant)
    proposed={e['token']:e for e in relevant if e['type']=='handoff_proposed'}
    authorized={e['token']:e for e in relevant if e['type']=='handoff_authorized'}
    consumed={e['token']:e for e in relevant if e['type']=='handoff_consumed'}
    cancelled={e['token']:e for e in relevant if e['type']=='handoff_cancelled'}
    latencies=[e['time']-proposed[token]['time'] for token,e in authorized.items() if token in proposed]
    # Legacy baseline has no request submission IDs. Its worker-running state
    # can explain idle residence, but cannot prove request/flight overlap.
    overlap_available=bool(requests) and unmatched==0
    handoffs=[dict(token=token,drone=e['drone'],proposed=e['time'],authorized=authorized.get(token,{}).get('time'),
                   consumed=consumed.get(token,{}).get('time'),cancelled=cancelled.get(token,{}).get('time'),
                   cancellation_reason=cancelled.get(token,{}).get('reason'),
                   speed_at_consumption=consumed.get(token,{}).get('speed'),
                   terminal_state='consumed' if token in consumed else 'cancelled' if token in cancelled else 'not_consumed_by_task_end')
              for token,e in proposed.items()]
    report=dict(schema='annual.execution-diagnostics/1',run=str(directory),mission_start=start,mission_end=end,
                low_speed_threshold_mps=low_speed,expected_fleet_s=(end-start)*summary['fleet_size'],
                classified_execution_fleet_s=sum(residence.values()),pose_speed_covered_fleet_s=motion_covered,
                actual_low_speed_fleet_s=total_low,actual_low_speed_fraction=total_low/motion_covered if motion_covered else None,
                actual_motion_fleet_s=total_high,maximum_sampled_pose_speed_mps=max_speed,
                residence_by_phase_fleet_s=dict(residence),low_speed_by_phase_fleet_s=dict(low),residence_by_uav_s=dict(by_uav),
                planning_request_counts=request_counts,unmatched_request_results=unmatched,
                request_overlap_status='RECORDED' if overlap_available else 'UNAVAILABLE_WITHOUT_COMPLETE_REQUEST_BOUNDARIES',
                planning_request_union_fleet_s=sum(b-a for value in requests.values() for a,b in value) if overlap_available else None,
                planning_execution_overlap_fleet_s=planning_overlap if overlap_available else None,
                planning_during_actual_motion_fleet_s=planning_during_motion if overlap_available else None,
                planning_during_low_speed_fleet_s=planning_during_low if overlap_available else None,
                handoff=dict(ready_views=counts['next_view_ready'],proposed=len(proposed),authorized=len(authorized),consumed=len(consumed),
                             proposed_to_consumed_ratio=len(consumed)/len(proposed) if proposed else None,
                             cancelled_reasons=dict(Counter(e.get('reason','unknown') for e in cancelled.values())),
                             proposal_to_authorization_sim_s=quantiles(latencies),attempts=handoffs,
                             rolling_latency_evidence=[e for e in relevant if e['type']=='handoff_preparation_timing']),
                limitations=['Low speed uses finite differences of sampled physical poses, not commanded reference speed.',
                             'Phase labels identify recorded execution states; low-speed tracking is not automatically planning delay.',
                             'Intervals with stale/missing execution or pose samples are unclassified, never silently zero.',
                             'Planning intervals include queue, compute and delivery. Parallel work is unioned per UAV, not summed as mission delay.',
                             'Ready views are not one-to-one with handoff proposals; ready-to-consumed is not an adoption success rate.',
                             'Legacy baseline lacks request submission IDs; request/flight overlap remains unavailable.'])
    inputs=[directory/n for n in ('summary.json','trajectory.csv','execution-evidence.jsonl','peer_states.jsonl')]+list(directory.glob('drone_*/events.jsonl'))
    def digest(path):
        h=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
        return h.hexdigest()
    report['input_sha256']={str(p.relative_to(directory)):digest(p) for p in inputs}
    report['tool_sha256']=digest(Path(__file__))
    (output/'diagnostics.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();r=diagnose(args.directory,args.output)
    print(json.dumps(dict(run=r['run'],low_speed_fraction=r['actual_low_speed_fraction'],handoffs=r['handoff']['consumed'])))
