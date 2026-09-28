#!/usr/bin/env python3
"""External native fault injection; never fabricates observations or odometry.

Run beside one normal no-fault run_todo_experiment invocation. Signals are
restricted to its exact GZ_PARTITION, installed script, namespace and PID start
time. The flight policy is neither modified nor switched by this probe.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time


def process(pid,partition):
    try:
        root=Path('/proc')/str(pid)
        environment=(root/'environ').read_bytes().split(b'\0')
        if ('GZ_PARTITION='+partition).encode() not in environment:return None
        fields=(root/'stat').read_text().rsplit(') ',1)[1].split()
        if fields[0]=='Z':return None
        return dict(pid=int(pid),parent=int(fields[1]),start_ticks=fields[19],
                    arguments=(root/'cmdline').read_bytes().decode().split('\0')[:-1])
    except (OSError,UnicodeError,ValueError,IndexError):return None


def processes(partition):
    return [value for root in Path('/proc').iterdir() if root.name.isdecimal()
            and (value:=process(root.name,partition))]


def target(partition,source,drone,worker):
    script=str(source/'ros2_ws/install/annual_swarm/lib/annual_swarm'/
               ('decentralized_agent_node.py' if worker else 'pointcloud_mapping_node.py'))
    inventory=processes(partition)
    nodes=[p for p in inventory if script in p['arguments'] and f'__ns:=/drone_{drone}' in p['arguments']]
    if len(nodes)!=1:return None
    if not worker:return nodes[0]
    children=[p for p in inventory if p['parent']==nodes[0]['pid']
              and any('multiprocessing.spawn' in argument for argument in p['arguments'])]
    return children[0] if len(children)==1 else None


def send(record,partition,signum):
    current=process(record['pid'],partition)
    if not current or current['start_ticks']!=record['start_ticks'] or current['arguments']!=record['arguments']:
        return False
    os.kill(record['pid'],signum);return True


def latest_states(root):
    result={};file=root/'peer_states.jsonl'
    if not file.exists():return result
    with file.open('rb') as stream:
        size=stream.seek(0,2);stream.seek(max(0,size-1048576))
        lines=stream.read().splitlines()
    for line in lines:
        try:value=json.loads(line);result[value['drone']]=value
        except (ValueError,KeyError):continue
    return result


def latest_request(root,drone):
    file=root/f'drone_{drone}/events.jsonl';request=None
    if not file.exists():return request
    for line in file.read_text().splitlines():
        value=json.loads(line)
        if value['type']=='planning_submitted':request=value['request_id']
        elif value.get('request_id')==request and value['type'] in ('planning_result','planning_failed','planning_timeout'):
            request=None
    return request


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--directory',required=True)
    parser.add_argument('--source-root',required=True);parser.add_argument('--wall-limit',type=float,default=7200.)
    args=parser.parse_args();root=Path(args.directory);source=Path(args.source_root).resolve()
    started=time.monotonic();partition=None;pending=None;index=0;outcomes=[]
    specifications=[dict(case='planning_deadline',drone=0,eligible=60.,worker=True,signum=signal.SIGSTOP,wall_hold=13.),
                    dict(case='planning_worker_crash',drone=1,eligible=100.,worker=True,signum=signal.SIGKILL,wall_hold=0.),
                    dict(case='perception_update_loss',drone=2,eligible=150.,worker=False,signum=signal.SIGSTOP,sim_hold=3.)]
    def record(value):
        value.update(wall_monotonic=time.monotonic());outcomes.append(value)
        with (root/'native-fault-events.jsonl').open('a') as stream:stream.write(json.dumps(value)+'\n')
    try:
        while time.monotonic()-started<args.wall_limit:
            if (root/'run-result.json').exists():break
            config=root/'run-configuration.json';summary=root/'summary.json'
            if not config.exists() or not summary.exists():time.sleep(.2);continue
            if partition is None:partition=json.loads(config.read_text())['environment']['GZ_PARTITION']
            try:s=json.loads(summary.read_text())
            except ValueError:time.sleep(.1);continue
            now=s['simulation_time']
            if pending:
                spec,identity,began,sim_started=pending
                due=now-sim_started>=spec['sim_hold'] if 'sim_hold' in spec else time.monotonic()-began>=spec['wall_hold']
                if due:
                    resumed=send(identity,partition,signal.SIGCONT) if spec['signum']==signal.SIGSTOP else False
                    record(dict(type='fault_window_ended',case=spec['case'],drone=spec['drone'],time=now,
                                original_process_alive=process(identity['pid'],partition) is not None,resumed=resumed))
                    pending=None;index+=1
            if index==len(specifications):break
            spec=specifications[index];state=latest_states(root).get(spec['drone'],{})
            intent=state.get('intent') or {};request=latest_request(root,spec['drone']) if spec['worker'] else None
            eligible=(not pending and now>=spec['eligible'] and intent.get('committed') and
                      (not spec['worker'] or state.get('fusion',{}).get('worker_running') and request))
            if eligible:
                identity=target(partition,source,spec['drone'],spec['worker'])
                if identity and send(identity,partition,spec['signum']):
                    record(dict(type='fault_injected',case=spec['case'],drone=spec['drone'],time=now,process=identity,
                                signal=signal.Signals(spec['signum']).name,request_id=request,
                                old_token=intent['token'],before_position=state['position']))
                    pending=(spec,identity,time.monotonic(),now)
            time.sleep(.1)
    finally:
        if pending and pending[0]['signum']==signal.SIGSTOP:send(pending[1],partition,signal.SIGCONT)
        if root.exists():(root/'native-fault-probe-result.json').write_text(json.dumps(dict(
            executed_cases=index,total_cases=len(specifications),events=outcomes,
            note='Injection completion is not a safety pass. Independently audit actual flight, leases, timeout/crash recovery and new sensor gains.'),indent=2)+'\n')


if __name__=='__main__':main()
