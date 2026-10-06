#!/usr/bin/env python3
"""Launch a real headless Gazebo mission, verify flight, contact and completion."""
import argparse
import json
import math
import os
import uuid
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--timeout',type=float,default=180)
    parser.add_argument('--goal',default='18,16,1.5'); parser.add_argument('--output-dir')
    parser.add_argument('--expect-no-safe-backup',action='store_true')
    parser.add_argument('--expect-backup-switch',action='store_true')
    parser.add_argument('--redirect-goal'); parser.add_argument('--redirect-after',type=float,default=12.)
    parser.add_argument('--expect-planner-failure',action='store_true')
    parser.add_argument('--planner',default='astar'); parser.add_argument('--controller',default='pid')
    parser.add_argument('--launch-arg',action='append',default=[])
    args=parser.parse_args()
    root=Path(args.output_dir or tempfile.mkdtemp(prefix='annual_smoke_')).resolve()
    output=root/f'run_{time.time_ns()}'; output.mkdir(parents=True,exist_ok=False)
    log=(output/'launch.log').open('w')
    environment=os.environ.copy(); environment['GZ_PARTITION']='annual_smoke_'+uuid.uuid4().hex
    process=subprocess.Popen(['ros2','launch','annual_swarm','swarm.launch.py','headless:=true',f'goal:={args.goal}',f'output_dir:={output}',f'planner:={args.planner}',f'controller:={args.controller}',*args.launch_arg],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=environment)
    goal=[float(x) for x in args.goal.split(',')]; deadline=time.monotonic()+args.timeout
    paused_since=None; pause_positions=None
    holding_since=None; redirect_process=None; redirected=False
    try:
        while time.monotonic()<deadline:
            if process.poll() is not None: raise RuntimeError(f'Launch exited {process.returncode}; see {output}/launch.log')
            summaries=sorted(output.glob('*/summary.json'))
            if summaries:
                summary=json.loads(summaries[-1].read_text())
                if args.redirect_goal and not redirected and summary['mission_status']=='FLYING' and summary['simulation_time']>=args.redirect_after:
                    goal=[float(x) for x in args.redirect_goal.split(',')]
                    payload=json.dumps({'header':{'frame_id':'world'},'pose':{'position':dict(zip(('x','y','z'),goal)),'orientation':{'w':1.}}})
                    redirect_process=subprocess.Popen(['ros2','topic','pub','--once','/swarm/goal','geometry_msgs/msg/PoseStamped',payload],env=environment,stdout=log,stderr=subprocess.STDOUT)
                    redirected=True
                if summary['mission_status'].startswith('FAILED'):
                    if not args.expect_planner_failure: raise AssertionError(summary)
                    if all(v>100 for v in summary['odometry_samples'].values()):
                        assert all(abs(p[2])<0.2 for p in summary['positions'].values()),summary
                        assert summary['samples']==0,summary
                        print(json.dumps(summary,indent=2)); print(f'PASS: rejected unsafe goal; {output}'); return
                    time.sleep(0.5); continue
                if summary['contact_samples_after_takeoff']>0: raise AssertionError(summary)
                if args.expect_no_safe_backup and summary['mission_status']=='PAUSED' and summary['planner'].get('map_version',0)>0:
                    if paused_since is None:
                        paused_since=summary['simulation_time'];pause_positions=summary['positions']
                    if summary['simulation_time']-paused_since>=3.:
                        assert summary['planner']['active'] is None and summary['planner']['backup_count']==0,summary
                        assert any(e['type']=='no_safe_backup' for e in summary['planner']['events']),summary
                        for i,p in summary['positions'].items():
                            assert 1.2<p[2]<1.8 and math.dist(p,pause_positions[i])<.5,summary
                        print(json.dumps(summary,indent=2));print(f'PASS: safe hover with no route; {output}');return
                if summary['mission_status']=='HOLDING':
                    assert not args.expect_planner_failure,summary
                    assert summary['planner'].get('algorithm')==args.planner,summary
                    assert len(summary['controllers'])==3 and all(x.startswith(args.controller+' ') for x in summary['controllers'].values()),summary
                    if args.planner=='dstar_lite' and any(x.startswith('replan_interval:=') and x!='replan_interval:=0.0' for x in args.launch_arg):
                        assert summary['planner'].get('dstar_state_reused'),summary
                    if holding_since is None: holding_since=summary['simulation_time']
                    if summary['simulation_time']-holding_since<3.0:
                        time.sleep(0.5); continue
                    if args.expect_backup_switch:
                        assert summary['planner'].get('initial_backup_count')==5,summary
                        assert summary['planner'].get('switch_count',0)>=1,summary
                        assert summary['planner'].get('map_version',0)>=1,summary
                        assert any(e['type']=='backup_switch' for e in summary['planner'].get('events',[])),summary
                    if any(x in args.launch_arg for x in ('apf:=true','formation_apf:=true')):
                        assert summary.get('formation',{}).get('apf_active_samples',0)>0,summary
                    if 'velocity_feedforward:=true' in args.launch_arg:
                        assert summary.get('formation',{}).get('max_reference_velocity',0)>.1,summary
                    if args.redirect_goal:
                        assert redirected and redirect_process.poll()==0,summary
                        assert summary['planner'].get('plan_count',0)>=2,summary
                        assert summary['planner'].get('goal')==goal,summary
                    assert all(v>100 for v in summary['odometry_samples'].values()),summary
                    assert all(v>100 for v in summary['imu_samples'].values()),summary
                    assert summary['min_pairwise_distance_m']>0.7,summary
                    for i,dy in enumerate([0,-1,1]):
                        position=summary['positions'][str(i)]
                        assert math.dist(position,[goal[0],goal[1]+dy,goal[2]])<0.3,summary
                    print(json.dumps(summary,indent=2)); print(f'PASS: {output}'); return
            time.sleep(0.5)
        raise TimeoutError(f'No completed flight within {args.timeout}s; see {output}')
    finally:
        # Signal launch once; it forwards shutdown to children. Signaling the
        # entire group here would deliver SIGINT twice to ROS nodes.
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        # gz's Ruby launcher may exit while the server remains in this process group.
        try: os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError: pass
        if redirect_process is not None and redirect_process.poll() is None:
            redirect_process.terminate(); redirect_process.wait(timeout=5)
        log.close()
if __name__=='__main__': main()
