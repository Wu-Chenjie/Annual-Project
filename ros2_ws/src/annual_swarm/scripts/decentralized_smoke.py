#!/usr/bin/env python3
"""Run and retain an auditable Gazebo decentralized exploration acceptance test."""
import argparse
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output-dir', required=True); p.add_argument('--timeout', type=float, default=1800.)
    p.add_argument('--map'); p.add_argument('--pause-after', type=float, default=60.)
    p.add_argument('--gui', action='store_true'); p.add_argument('--record', action='store_true')
    args = p.parse_args(); out = Path(args.output_dir).resolve(); out.mkdir(parents=True, exist_ok=False)
    cmd = ['ros2', 'launch', 'annual_swarm', 'decentralized_search.launch.py', f'headless:={str(not args.gui).lower()}',
           f'rviz:={str(args.gui).lower()}', f'visualize:={str(args.gui).lower()}', f'output_dir:={out}', f'pause_after:={args.pause_after}']
    if args.map:
        cmd.append(f'map:={args.map}')
    env = os.environ.copy(); env['GZ_PARTITION'] = 'annual_decentralized_'+uuid.uuid4().hex
    recorder = None
    with (out/'launch.log').open('w') as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+args.timeout; holding = None
        try:
            if args.record:
                (out/'recording-start.json').write_text(json.dumps(dict(wall_start_unix=time.time(), gz_partition=env['GZ_PARTITION'], width=1920, height=1080, fps=10)))
                recorder = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'warning', '-f', 'x11grab', '-framerate', '10',
                    '-video_size', '1920x1080', '-i', os.environ.get('DISPLAY', ':99'), '-c:v', 'libx264', '-preset', 'ultrafast',
                    '-crf', '23', '-threads', '1', '-pix_fmt', 'yuv420p', str(out/'gazebo-rviz-raw.mp4')], stdout=log, stderr=log)
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'Launch exited, see {out}')
                file = out/'summary.json'
                if file.exists():
                    if time.time()-file.stat().st_mtime > 20:
                        raise RuntimeError('Experiment telemetry stopped; inspect launch.log')
                    s = json.loads(file.read_text())
                    assert s['contacts_after_takeoff'] == 0 and s['status'] != 'FAILED', s
                    if s['status'] == 'COMPLETE':
                        if holding is None:
                            holding = s['simulation_time']
                        if s['simulation_time']-holding >= 3:
                            assert s['coverage'] >= .95 and s['min_separation_m'] > .8, s
                            assert all(d > 3 for d in s['distances_m'].values()), s
                            assert all(n > 0 for n in s['views'].values()), s
                            assert all(g['nodes'] < g['free_cells']/4 for g in s['graph'].values()), s
                            if args.pause_after:
                                assert s['pause_resumed'], s
                                events = [json.loads(line) for file in out.glob('drone_*/events.jsonl') for line in file.read_text().splitlines()]
                                assert any(e['type'] == 'region_reassigned' for e in events), 'No observed peer-driven region reassignment'
                            print(json.dumps(s, indent=2)); return
                time.sleep(1)
            raise TimeoutError(str(out))
        finally:
            if recorder and recorder.poll() is None:
                recorder.send_signal(signal.SIGINT); recorder.wait(timeout=20)
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            # Gazebo's GUI/server launcher may create additional process groups.
            # Match only this run's unique transport partition, never other sims.
            owned = []
            if Path('/proc').is_dir():
                marker = ('GZ_PARTITION='+env['GZ_PARTITION']).encode()
                for entry in Path('/proc').iterdir():
                    if not entry.name.isdigit():
                        continue
                    try:
                        if marker in (entry/'environ').read_bytes().split(b'\0'):
                            owned.append(int(entry.name))
                    except (FileNotFoundError, PermissionError, ProcessLookupError):
                        pass
                for pid in owned:
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                time.sleep(.5)
                for pid in owned:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

if __name__ == '__main__':
    main()
