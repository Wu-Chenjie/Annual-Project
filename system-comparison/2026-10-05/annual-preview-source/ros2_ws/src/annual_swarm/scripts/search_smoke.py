#!/usr/bin/env python3
"""A full real-physics search mission, optionally pausing a worker to test transfer."""
import argparse,json,os,signal,subprocess,time,uuid
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',required=True);p.add_argument('--timeout',type=float,default=900)
    p.add_argument('--map');p.add_argument('--policy',default='gvp_pairwise');p.add_argument('--pause-after',type=float,default=0.)
    args=p.parse_args();out=Path(args.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    cmd=['ros2','launch','annual_swarm','search.launch.py','headless:=true',f'output_dir:={out}',f'policy:={args.policy}',f'pause_after:={args.pause_after}']
    if args.map:cmd.append(f'map:={args.map}')
    env=os.environ.copy();env['GZ_PARTITION']='annual_search_'+uuid.uuid4().hex
    with (out/'launch.log').open('w') as log:
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+args.timeout;holding=None
        try:
            while time.monotonic()<deadline:
                if proc.poll() is not None:raise RuntimeError(f'Launch exited, see {out}')
                path=out/'summary.json'
                if path.exists():
                    s=json.loads(path.read_text());assert s['contacts_after_takeoff']==0,s
                    assert s['status']!='FAILED',s
                    if s['status']=='COMPLETE':
                        if holding is None:holding=s['simulation_time']
                        if s['simulation_time']-holding>=3:
                            assert s['coverage']>=.95 and s['min_separation_m']>.8,s
                            assert all(n>100 for n in s['odometry_samples'].values()),s
                            assert all(d>3 for d in s['distances_m'].values()),s
                            if args.pause_after:
                                assert s['pause_resumed'] and any(e['type']=='task_transferred' for e in s['events']),s
                            print(json.dumps({k:s[k] for k in ['status','coverage','t90','t95','min_separation_m','distances_m']},indent=2));return
                time.sleep(1)
            raise TimeoutError(str(out))
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
if __name__=='__main__':main()
