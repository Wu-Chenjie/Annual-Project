#!/usr/bin/env python3
"""Run real Gazebo missions for every planner and controller; retain each result."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

CASES=[
    ('astar','pid',[]),('heading_astar','smc',[]),
    ('hybrid_astar','backstepping',[]),('dijkstra','backstepping_pid',[]),
    ('rrt_star','super_twisting',[]),('informed_rrt_star','geometric_euler',[]),
    ('dstar_lite','pid',['replan_interval:=5.0']),('gnn','smc',[]),
    ('window','backstepping',['replan_interval:=3.0']),
    ('portfolio','pid',['dynamic_obstacle:=true','dynamic_demo:=true']),
    ('astar','pid',['esdf:=true','firi:=true','trajectory:=minimum_jerk',
                    'apf:=true','formation_apf:=true','velocity_feedforward:=true'])]

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output-dir',required=True)
    parser.add_argument('--timeout',type=float,default=240)
    args=parser.parse_args(); root=Path(args.output_dir).resolve(); root.mkdir(parents=True,exist_ok=True)
    results=[]
    for index,(planner,controller,options) in enumerate(CASES):
        folder=root/f'{index:02d}_{planner}_{controller}'; folder.mkdir(exist_ok=True)
        command=[sys.executable,str(Path(__file__).with_name('smoke_test.py')),
            '--planner',planner,'--controller',controller,'--timeout',str(args.timeout),'--output-dir',str(folder)]
        if planner=='portfolio':command+=['--expect-backup-switch']
        for option in options: command+=['--launch-arg',option]
        print('RUN',planner,controller,options,flush=True)
        with (folder/'test.log').open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        results.append(dict(planner=planner,controller=controller,options=options,returncode=result.returncode))
        (root/'results.json').write_text(json.dumps(results,indent=2)); print(results[-1],flush=True)
    return int(any(x['returncode'] for x in results))
if __name__=='__main__': sys.exit(main())
