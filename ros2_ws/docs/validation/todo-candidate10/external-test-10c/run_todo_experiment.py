#!/usr/bin/env python3
"""One preregistered isolated run. Every timeout/failure is retained."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from experiment_processes import terminate_partition


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--map', required=True)
    parser.add_argument('--faults', choices=['none', 'recovery'], default='none')
    parser.add_argument('--simulation-limit', type=float, default=2400.)
    parser.add_argument('--wall-limit', type=float, default=7200.)
    parser.add_argument('--observer', default=str(Path(__file__).with_name('record_observation_evidence.py')))
    args = parser.parse_args()
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', LP_NUM_THREADS='2',
               LIBGL_ALWAYS_SOFTWARE='1', QT_X11_NO_MITSHM='1', PYTHONDONTWRITEBYTECODE='1',
               ANNUAL_EXPERIMENT_SEED=str(args.seed), GZ_PARTITION='annual_todo_'+uuid.uuid4().hex,
               ROS_DOMAIN_ID=str(20+int(uuid.uuid4().hex[:4], 16)%180))
    (output/'run-configuration.json').write_text(json.dumps(dict(arguments=vars(args),
        map_sha256=hashlib.sha256(Path(args.map).read_bytes()).hexdigest(),
        environment={k:env[k] for k in ('ANNUAL_EXPERIMENT_SEED', 'GZ_PARTITION', 'ROS_DOMAIN_ID')},
        cpu_assignment=6, memory_assignment_gib=8), indent=2)+'\n')
    commands = ['ros2', 'launch', 'annual_swarm', 'decentralized_search.launch.py',
                'headless:=true', 'rviz:=false', 'visualize:=false', f'map:={args.map}', f'output_dir:={output}']
    if args.faults == 'none':
        commands += ['pause_after:=0', 'network_after:=0', 'restart_after:=0', 'dynamic_obstacle:=false']
    log = (output/'launch.log').open('w'); started = time.monotonic(); last = None; result = {}; bad_since={}
    observer = subprocess.Popen(['python3', args.observer, '--ros-args', '-p', 'use_sim_time:=true',
        '-p', f'output_dir:={output}'], env=env, stdout=log, stderr=log, start_new_session=True)
    launch = subprocess.Popen(commands, env=env, stdout=log, stderr=log, start_new_session=True)
    try:
        while time.monotonic()-started < args.wall_limit:
            if launch.poll() is not None or observer.poll() is not None:
                raise RuntimeError('Launch or evidence recorder exited')
            failures = (output/'launch.log').read_text(errors='replace')[-50000:]
            if 'pointcloud_mapping_node.py' in failures and 'TypeError:' in failures:
                raise RuntimeError('Observation instrumentation failed; check launch.log')
            path = output/'summary.json'
            if path.exists():
                try: last = json.loads(path.read_text())
                except (json.JSONDecodeError, FileNotFoundError): continue
                if last['status'] in ('COMPLETE', 'FAILED'):
                    result = dict(outcome=last['status'], reason=last.get('failure_reason')); break
                if last.get('start_time') is not None and last['simulation_time']>last['start_time']+5.:
                    peers=output/'peer_states.jsonl'
                    if peers.exists():
                        with peers.open('rb') as stream:
                            size=stream.seek(0,2);stream.seek(max(0,size-262144));lines=stream.read().splitlines()
                        latest={}
                        for line in lines:
                            try:state=json.loads(line)
                            except (json.JSONDecodeError,UnicodeDecodeError):continue
                            latest[state['drone']]=state
                        for i,state in latest.items():
                            bad=state['position'][2]<.5 or state.get('execution',{}).get('tracking_error',0.)>.8
                            if bad:
                                bad_since.setdefault(i,state['time'])
                                if state['time']-bad_since[i]>=1.:
                                    result=dict(outcome='EXECUTION_FAILURE',reason='Sustained landing or tracking loss',drone=i,time=state['time'])
                            else:bad_since.pop(i,None)
                    if max(last.get('agent_ages',{}).values(),default=0)>15.:
                        result=dict(outcome='AGENT_FAILURE',reason='Exploration heartbeat stopped for 15 simulation seconds')
                    if result:break
                if time.time()-path.stat().st_mtime > 90.:
                    raise RuntimeError('Experiment telemetry stopped')
                if last.get('start_time') is not None and last['simulation_time']-last['start_time'] >= args.simulation_limit:
                    result = dict(outcome='SIMULATION_TIME_LIMIT'); break
            elif time.monotonic()-started > 180.:
                raise RuntimeError('Experiment did not start')
            time.sleep(1.)
        if not result: result = dict(outcome='WALL_TIME_LIMIT')
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(outcome='INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'INFRASTRUCTURE_FAILURE', reason=repr(exc))
    finally:
        result.update(wall_elapsed_s=time.monotonic()-started,
            last_coverage=last.get('coverage') if last else None,
            last_simulation_time=last.get('simulation_time') if last else None)
        (output/'run-result.json').write_text(json.dumps(result, indent=2)+'\n')
        for process in (launch, observer):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
        cleanup=terminate_partition(env['GZ_PARTITION'])
        (output/'process-cleanup.json').write_text(json.dumps(cleanup,indent=2)+'\n')
        if cleanup['remaining']:raise RuntimeError('Owned experiment processes survived cleanup')
        log.close()
    print(json.dumps(result), flush=True)


if __name__ == '__main__': main()
