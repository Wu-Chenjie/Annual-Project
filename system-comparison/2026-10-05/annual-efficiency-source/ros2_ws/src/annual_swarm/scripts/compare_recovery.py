#!/usr/bin/env python3
"""Sequential, counterbalanced Gazebo recovery-policy comparison; no mocked flights."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

POLICIES=('cache','replan_single','replan_pool')

def measure(root,policy):
    summary_path=next(root.glob('run_*/*/summary.json'))
    s=json.loads(summary_path.read_text());p=s['planner']
    assert p['initial_backup_count']==5 and p['recovery_policy']==policy
    assert s['mission_status']=='HOLDING' and s['contact_samples_after_takeoff']==0
    events=[e for e in p['events'] if e['type']=='recovery_ready']
    assert len(events)==1 and events[0]['map_version']==1
    assert p['switch_count']==(1 if policy=='cache' else 0)
    obstacle=s['dynamic_obstacles']
    assert obstacle['obstacles'][0]['center_xy']==[9.5,5.]
    assert 3.0<=obstacle['trigger_leader_position'][0]<3.2
    if policy=='replan_single':assert p['active']['planner']=='astar'
    snapshots=[json.loads(x) for x in (summary_path.parent/'candidate_paths.jsonl').read_text().splitlines()]
    initial=next(x for x in snapshots if x['active_path'])
    transitions=s['state_transitions']
    flying=next(x['simulation_time'] for x in transitions if x['status']=='FLYING')
    holding=next(x['simulation_time'] for x in transitions if x['status']=='HOLDING')
    pause=sum(b['simulation_time']-a['simulation_time'] for a,b in zip(transitions,transitions[1:]) if a['status']=='PAUSED')
    rows=list(csv.DictReader((summary_path.parent/'trajectory.csv').open()))
    points=[(float(r['x']),float(r['y'])) for r in rows if r['drone']=='0' and flying<=float(r['time'])<=holding]
    return dict(policy=policy,summary=str(summary_path),initial_path_sha256=hashlib.sha256(json.dumps(initial['active_path']).encode()).hexdigest(),
        obstacle=s['dynamic_obstacles'],recovery_wall_s=events[0]['latency_wall_s'],pause_sim_s=pause,
        flight_sim_s=holding-flying,leader_distance_xy_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
        contacts=s['contact_samples_after_takeoff'],min_separation_m=s['min_pairwise_distance_m'])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--repeats',type=int,default=2);parser.add_argument('--output-dir',required=True)
    args=parser.parse_args();assert args.repeats>0
    output=Path(args.output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
    results=[]
    for repeat in range(args.repeats):
        order=POLICIES if repeat%2==0 else POLICIES[::-1]
        for policy in order:
            root=output/f'{policy}_{repeat+1}';root.mkdir()
            cmd=[sys.executable,str(Path(__file__).with_name('smoke_test.py')),'--planner','portfolio','--timeout','240','--output-dir',str(root)]
            for arg in ('dynamic_obstacle:=true','dynamic_demo:=true','dynamic_demo_mode:=benchmark',f'recovery_policy:={policy}'):
                cmd+=['--launch-arg',arg]
            print(f'RUN {repeat+1}/{args.repeats} {policy}',flush=True)
            with (root/'smoke.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
            row=measure(root,policy);row['repeat']=repeat+1;results.append(row)
            assert len({r['initial_path_sha256'] for r in results})==1,'Initial routes differ'
            (output/'results.json').write_text(json.dumps(results,indent=2))
            print(json.dumps(row),flush=True)
    print('PASS: all controlled flights completed',flush=True)
if __name__=='__main__':main()
